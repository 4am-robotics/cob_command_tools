#!/usr/bin/env python
#
# Copyright 2017 Fraunhofer Institute for Manufacturing Engineering and Automation (IPA)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#   http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.




import sys
import traceback
import subprocess
import socket
import psutil
import numpy as np
import math

import rospy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

from netdata_tools.netdata_tools import query_netdata_info, query_netdata

stat_dict = { DiagnosticStatus.OK: 'OK', DiagnosticStatus.WARN: 'Warning', DiagnosticStatus.ERROR: 'Error', DiagnosticStatus.STALE: 'Stale' }

class CPUMonitor():
    def __init__(self, hostname, diag_hostname):
        self._check_ipmi = rospy.get_param('~check_ipmi_tool', False)
        self._check_core_temps = rospy.get_param('~check_core_temps', False)

        self._core_load_warn = rospy.get_param('~core_load_warn', 90)
        self._core_load_error = rospy.get_param('~core_load_error', 110)
        self._load1_threshold = rospy.get_param('~load1_threshold', 5.0)
        self._load5_threshold = rospy.get_param('~load5_threshold', 3.0)
        self._core_temp_warn = rospy.get_param('~core_temp_warn', 90)
        self._core_temp_error = rospy.get_param('~core_temp_error', 95)
        self._mem_warn = rospy.get_param('~mem_warn', 25)
        self._mem_error = rospy.get_param('~mem_error', 1)

        self._check_thermal_throttling_events = rospy.get_param('~check_thermal_throttling_events', False)
        self._thermal_throttling_threshold = rospy.get_param('~thermal_throttling_threshold', 1000)

        self._check_idlejitter = rospy.get_param('~check_idlejitter', False)
        self._idlejitter_min_threshold = rospy.get_param('~idlejitter_min_threshold', 50000)
        self._idlejitter_max_threshold = rospy.get_param('~idlejitter_max_threshold', 2000000)
        self._idlejitter_average_threshold = rospy.get_param('~idlejitter_average_threshold', 200000)

        self._num_cores = rospy.get_param('~num_cores', psutil.cpu_count())

        # CPU stats
        self._info_stat = DiagnosticStatus()
        self._info_stat.name = '%s CPU Info' % diag_hostname
        self._info_stat.level = DiagnosticStatus.WARN
        self._info_stat.hardware_id = hostname
        self._info_stat.message = 'No Data'
        self._info_stat.values = []

        self._usage_stat = DiagnosticStatus()
        self._usage_stat.name = '%s CPU Usage' % diag_hostname
        self._usage_stat.level = DiagnosticStatus.WARN
        self._usage_stat.hardware_id = hostname
        self._usage_stat.message = 'No Data'
        self._usage_stat.values = []

        self._memory_stat = DiagnosticStatus()
        self._memory_stat.name = '%s Memory Usage' % diag_hostname
        self._memory_stat.level = DiagnosticStatus.WARN
        self._memory_stat.hardware_id = hostname
        self._memory_stat.message = 'No Data'
        self._memory_stat.values = []

        self._diag_pub = rospy.Publisher('/diagnostics', DiagnosticArray, queue_size=1)
        self._publish_timer = rospy.Timer(rospy.Duration(1.0), self.publish_stats)
        self._info_timer = rospy.Timer(rospy.Duration(5.0), self.check_info)
        self._usage_timer = rospy.Timer(rospy.Duration(5.0), self.check_usage)
        self._memory_timer = rospy.Timer(rospy.Duration(5.0), self.check_memory)

    ##\brief Output entire IPMI data set
    def check_ipmi(self):
        diag_vals = []
        diag_msgs = []
        diag_level = DiagnosticStatus.OK

        try:
            p = subprocess.Popen('sudo ipmitool sdr',
                                 stdout = subprocess.PIPE,
                                 stderr = subprocess.PIPE, shell = True)
            stdout, stderr = p.communicate()
            retcode = p.returncode
            try:
                stdout = stdout.decode()  #python3
            except (UnicodeDecodeError, AttributeError):
                pass

            if retcode != 0:
                diag_level = DiagnosticStatus.ERROR
                diag_msgs = [ 'ipmitool Error' ]
                diag_vals = [ KeyValue(key = 'IPMI Error', value = stderr) ,
                              KeyValue(key = 'Output', value = stdout) ]
                return diag_vals, diag_msgs, diag_level

            lines = stdout.split('\n')
            if len(lines) < 2:
                diag_vals = [ KeyValue(key = 'ipmitool status', value = 'No output') ]

                diag_msgs = [ 'No ipmitool response' ]
                diag_level = DiagnosticStatus.ERROR

                return diag_vals, diag_msgs, diag_level

            for ln in lines:
                if len(ln) < 3:
                    continue

                words = ln.split('|')
                if len(words) < 3:
                    continue

                name = words[0].strip()
                ipmi_val = words[1].strip()
                #stat_byte = words[2].strip()

                # CPU temps
                if words[0].startswith('CPU') and words[0].strip().endswith('Temp'):
                    if words[1].strip().endswith('degrees C'):
                        tmp = ipmi_val.rstrip(' degrees C').lstrip()
                        try:
                            temperature = float(tmp)
                            diag_vals.append(KeyValue(key = name + ' (C)', value = tmp))

                            #cpu_name = name.split()[0]
                            if temperature >= self._core_temp_error: # CPU should shut down here
                                diag_level = max(diag_level, DiagnosticStatus.ERROR)
                                diag_msgs.append('CPU Hot')
                            elif temperature >= self._core_temp_warn:
                                diag_level = max(diag_level, DiagnosticStatus.WARN)
                                diag_msgs.append('CPU Warm')
                        except ValueError:
                            diag_level = max(diag_level, DiagnosticStatus.ERROR)
                            diag_msgs.append('Error: temperature not numeric')
                    else:
                        diag_vals.append(KeyValue(key = name, value = words[1]))


                # MP, BP, FP temps
                if name == 'MB Temp' or name == 'BP Temp' or name == 'FP Temp':
                    if ipmi_val.endswith('degrees C'):
                        tmp = ipmi_val.rstrip(' degrees C').lstrip()
                        diag_vals.append(KeyValue(key = name + ' (C)', value = tmp))
                        # Give temp warning
                        dev_name = name.split()[0]
                        try:
                            temperature = float(tmp)

                            if temperature >= 60 and temperature < 75:
                                diag_level = max(diag_level, DiagnosticStatus.WARN)
                                diag_msgs.append('%s Warm' % dev_name)

                            if temperature >= 75:
                                diag_level = max(diag_level, DiagnosticStatus.ERROR)
                                diag_msgs.append('%s Hot' % dev_name)
                        except ValueError:
                            diag_level = max(diag_level, DiagnosticStatus.ERROR)
                            diag_msgs.append('%s Error: temperature not numeric' % dev_name)
                    else:
                        diag_vals.append(KeyValue(key = name, value = ipmi_val))

                # CPU fan speeds
                if (name.startswith('CPU') and name.endswith('Fan')) or name == 'MB Fan':
                    if ipmi_val.endswith('RPM'):
                        rpm = ipmi_val.rstrip(' RPM').lstrip()
                        try:
                            if int(rpm) == 0:
                                diag_level = max(diag_level, DiagnosticStatus.ERROR)
                                diag_msgs.append('CPU Fan Off')

                            diag_vals.append(KeyValue(key = name + ' RPM', value = rpm))
                        except ValueError:
                            diag_vals.append(KeyValue(key = name, value = ipmi_val))

                # If CPU is hot we get an alarm from ipmitool, report that too
                # CPU should shut down if we get a hot alarm, so report as error
                if name.startswith('CPU') and name.endswith('hot'):
                    if ipmi_val == '0x01':
                        diag_vals.append(KeyValue(key = name, value = 'OK'))
                    else:
                        diag_vals.append(KeyValue(key = name, value = 'Hot'))
                        diag_level = max(diag_level, DiagnosticStatus.ERROR)
                        diag_msgs.append('CPU Hot Alarm')

        except Exception as e:
            diag_level = DiagnosticStatus.ERROR
            diag_msgs = [ 'IPMI Exception' ]
            diag_vals = [ KeyValue(key = 'Exception', value = str(e)), KeyValue(key = 'Traceback', value = str(traceback.format_exc())) ]

        return diag_vals, diag_msgs, diag_level

    ##\brief Check CPU core temps
    def check_core_temps(self, interval=1):
        diag_vals = []
        diag_msgs = []
        diag_level = DiagnosticStatus.OK

        try:
            netdata_core_temp = query_netdata('sensors.coretemp_isa_0000_temperature', interval)
            del netdata_core_temp['time']
            
            if not netdata_core_temp:
                diag_level = DiagnosticStatus.ERROR
                diag_msgs = [ 'Core Temp Error' ]
                diag_vals = [ KeyValue(key = 'Core Temp Error', value = 'Could not fetch data from netdata'),
                                KeyValue(key = 'Output', value = str(netdata_core_temp)) ]
                return diag_vals, diag_msgs, diag_level

            for core_no, values in netdata_core_temp.items():

                mean_temp = np.mean(values)
                try:
                    diag_vals.append(KeyValue(key = 'Temp %s' % core_no, value = str(mean_temp)))

                    if mean_temp >= self._core_temp_error:
                        diag_level = max(diag_level, DiagnosticStatus.OK) #do not set ERROR
                        diag_msgs.append('CPU Hot')
                    elif mean_temp >= self._core_temp_warn:
                        diag_level = max(diag_level, DiagnosticStatus.OK) #do not set WARN
                        diag_msgs.append('CPU Warm')
                except ValueError:
                    diag_level = max(diag_level, DiagnosticStatus.ERROR) # Error if not numeric value
                    diag_vals.append(KeyValue(key = 'Temp %s' % core_no, value = str(mean_temp)))

        except Exception as e:
            diag_level = DiagnosticStatus.ERROR
            diag_msgs = [ 'Core Temp Exception' ]
            diag_vals = [ KeyValue(key = 'Exception', value = str(e)), KeyValue(key = 'Traceback', value = str(traceback.format_exc())) ]

        return diag_vals, diag_msgs, diag_level

    ##\brief Checks clock speed from reading from CPU info
    def check_clock_speed(self, interval=1):
        diag_vals = []
        diag_msgs = []
        diag_level = DiagnosticStatus.OK

        try:
            netdata_cpu_freq = query_netdata('cpu.cpufreq', interval)
            del netdata_cpu_freq["time"]

            if not netdata_cpu_freq:
                diag_level = DiagnosticStatus.ERROR
                diag_msgs = [ 'Clock Speed Error' ]
                diag_vals = [ KeyValue(key = 'Clock Speed Error', value = 'Could not fetch data from netdata'),
                              KeyValue(key = 'Output', value = str(netdata_cpu_freq)) ]
                return (diag_vals, diag_msgs, diag_level)

            for cpu_name, values in netdata_cpu_freq.items():
                diag_vals.append(KeyValue(key = 'Core %d (MHz)' % int(cpu_name[-1]), value = str(np.mean(values))))

            # get max freq
            p = subprocess.Popen('lscpu | grep "max MHz"',
                                 stdout = subprocess.PIPE,
                                 stderr = subprocess.PIPE, shell = True)
            stdout, stderr = p.communicate()
            retcode = p.returncode
            try:
                stdout = stdout.decode()  #python3
            except (UnicodeDecodeError, AttributeError):
                pass

            if retcode != 0:
                diag_level = DiagnosticStatus.ERROR
                diag_msgs = [ 'Clock Speed Error' ]
                diag_vals = [ KeyValue(key = 'Clock Speed Error', value = stderr),
                              KeyValue(key = 'Output', value = stdout) ]
                return (diag_vals, diag_msgs, diag_level)

            diag_vals.append(KeyValue(key = stdout.split(':')[0].strip(), value = str(stdout.split(':')[1].strip())))

            # get min freq
            p = subprocess.Popen('lscpu | grep "min MHz"',
                                 stdout = subprocess.PIPE,
                                 stderr = subprocess.PIPE, shell = True)
            stdout, stderr = p.communicate()
            retcode = p.returncode
            try:
                stdout = stdout.decode()  #python3
            except (UnicodeDecodeError, AttributeError):
                pass

            if retcode != 0:
                diag_level = DiagnosticStatus.ERROR
                diag_msgs = [ 'Clock Speed Error' ]
                diag_vals = [ KeyValue(key = 'Clock Speed Error', value = stderr),
                              KeyValue(key = 'Output', value = stdout) ]
                return (diag_vals, diag_msgs, diag_level)

            diag_vals.append(KeyValue(key = stdout.split(':')[0].strip(), value = str(stdout.split(':')[1].strip())))

        except Exception as e:
            diag_level = DiagnosticStatus.ERROR
            diag_msgs = [ 'Clock Speed Exception' ]
            diag_vals = [ KeyValue(key = 'Exception', value = str(e)), KeyValue(key = 'Traceback', value = str(traceback.format_exc())) ]

        return diag_vals, diag_msgs, diag_level

    ##\brief Uses 'uptime' to see load average
    def check_uptime(self, interval=1):
        diag_vals = []
        diag_msg = ''
        diag_level = DiagnosticStatus.OK

        load_dict = { DiagnosticStatus.OK: 'OK', DiagnosticStatus.WARN: 'High Load', DiagnosticStatus.ERROR: 'Very High Load' }

        try:
            netdata_uptime = query_netdata('system.uptime', interval)
            del netdata_uptime['time']

            if not netdata_uptime:
                diag_level = DiagnosticStatus.ERROR
                diag_msg = 'Uptime Error'
                diag_vals = [ KeyValue(key = 'Uptime Error', value = 'Could not fetch data from netdata'),
                              KeyValue(key = 'Output', value = str(netdata_uptime)) ]
                return (diag_vals, diag_msg, diag_level)
            
            diag_vals.append(KeyValue(key = 'Uptime', value = str(np.max(netdata_uptime['uptime'].astype(float)))))

            netdata_cpu_load = query_netdata('system.load', interval)
            del netdata_cpu_load['time']

            if not netdata_cpu_load:
                diag_level = DiagnosticStatus.ERROR
                diag_msg = 'Load Error'
                diag_vals = [ KeyValue(key = 'Load Error', value = 'Could not fetch data from netdata'),
                              KeyValue(key = 'Output', value = str(netdata_cpu_load)) ]
                return (diag_vals, diag_msg, diag_level)

            load1 = np.mean(netdata_cpu_load['load1'].astype(float))
            load5 = np.mean(netdata_cpu_load['load5'].astype(float))
            load15 = np.mean(netdata_cpu_load['load15'].astype(float))

            # Give warning if we go over load limit
            if float(load1) > self._load1_threshold or float(load5) > self._load5_threshold:
                diag_level = DiagnosticStatus.WARN

            diag_vals.append(KeyValue(key = 'Load Average Status', value = load_dict[diag_level]))
            diag_vals.append(KeyValue(key = '1 min Load Average', value = str(load1)))
            diag_vals.append(KeyValue(key = '1 min Load Average Threshold', value = str(self._load1_threshold)))
            diag_vals.append(KeyValue(key = '5 min Load Average', value = str(load5)))
            diag_vals.append(KeyValue(key = '5 min Load Average Threshold', value = str(self._load5_threshold)))
            diag_vals.append(KeyValue(key = '15 min Load Average', value = str(load15)))

            diag_msg = load_dict[diag_level]

        except Exception as e:
            diag_level = DiagnosticStatus.ERROR
            diag_msg = 'Uptime Exception'
            diag_vals = [ KeyValue(key = 'Exception', value = str(e)), KeyValue(key = 'Traceback', value = str(traceback.format_exc())) ]

        return diag_vals, diag_msg, diag_level

    ##\brief Uses 'free -m' to check free memory
    def check_free_memory(self, interval=1):
        diag_vals = []
        diag_msg = ''
        diag_level = DiagnosticStatus.OK

        mem_dict = { DiagnosticStatus.OK: 'OK', DiagnosticStatus.WARN: 'Low Memory', DiagnosticStatus.ERROR: 'Very Low Memory' }

        try:
            netdata_mem = query_netdata('system.ram', interval)
            del netdata_mem['time']

            if not netdata_mem:
                diag_level = DiagnosticStatus.ERROR
                diag_msg = 'Memory Usage Error'
                diag_vals = [ KeyValue(key = 'Memory Usage Error', value = 'Could not fetch data from netdata'),
                              KeyValue(key = 'Output', value = str(netdata_mem)) ]
                return (diag_vals, diag_msg, diag_level)

            # Mem
            memory_vals = {k: np.mean(v.astype(float)) for k, v in netdata_mem.items()}
            total_mem = sum([val for val in memory_vals.values()])
            free_mem = memory_vals['free']
            used_mem = memory_vals['used']
            cache_mem = memory_vals['cached'] + memory_vals['buffers']

            diag_level = DiagnosticStatus.OK
            if float(free_mem) < self._mem_warn:
                diag_level = DiagnosticStatus.WARN
            if float(free_mem) < self._mem_error:
                diag_level = DiagnosticStatus.ERROR

            diag_vals.append(KeyValue(key = 'Mem Status', value = mem_dict[diag_level]))
            diag_vals.append(KeyValue(key = 'Mem Total', value = str(total_mem)))
            diag_vals.append(KeyValue(key = 'Mem Used', value = str(used_mem)))
            diag_vals.append(KeyValue(key = 'Mem Free', value = str(free_mem)))
            diag_vals.append(KeyValue(key = 'Mem Buff/Cache', value = str(cache_mem)))


            netdata_swp = query_netdata('system.swap', interval)
            del netdata_swp['time']

            if not netdata_swp:
                diag_level = DiagnosticStatus.ERROR
                diag_msg = 'Swap Usage Error'
                diag_vals = [ KeyValue(key = 'Swap Usage Error', value = 'Could not fetch data from netdata'),
                              KeyValue(key = 'Output', value = str(netdata_swp)) ]
                return (diag_vals, diag_msg, diag_level)

            # Swap
            swap_vals = {k: np.mean(v.astype(float)) for k, v in netdata_swp.items()}
            total_swp = sum([val for val in swap_vals.values()])
            free_swp = swap_vals['free']
            used_swp = swap_vals['used']

            diag_vals.append(KeyValue(key = 'Swap Total', value = str(total_swp)))
            diag_vals.append(KeyValue(key = 'Swap Used', value = str(used_swp)))
            diag_vals.append(KeyValue(key = 'Swap Free', value = str(free_swp)))

            diag_msg = mem_dict[diag_level]

        except Exception as e:
            diag_level = DiagnosticStatus.ERROR
            diag_msg = 'Memory Usage Exception'
            diag_vals = [ KeyValue(key = 'Exception', value = str(e)), KeyValue(key = 'Traceback', value = str(traceback.format_exc())) ]

        return diag_vals, diag_msg, diag_level


    def check_cpu_util(self, interval=1):
        diag_vals = []
        diag_msg = ''
        diag_level = DiagnosticStatus.OK

        load_dict = { DiagnosticStatus.OK: 'OK', DiagnosticStatus.WARN: 'High Load', DiagnosticStatus.ERROR: 'Error' }

        try:
            netdata_info = query_netdata_info()
            num_cores = int(netdata_info['cores_total'])

            netdata_system_cpu = query_netdata('system.cpu', interval)
            netdata_cpu_util = [query_netdata('cpu.cpu%d' % i, interval) for i in range(num_cores)]
            netdata_cpu_idle = [query_netdata('cpu.cpu%d_cpuidle' % i, interval) for i in range(num_cores)]


            if any([i == None for i in [netdata_system_cpu, netdata_cpu_util, netdata_cpu_idle]]):
                diag_level = DiagnosticStatus.ERROR
                diag_msg = 'CPU Usage Error'
                diag_vals = [ KeyValue(key = 'CPU Usage Error', value = 'Could not fetch data from netdata'),
                              KeyValue(key = 'Output', value = str([netdata_system_cpu, netdata_cpu_util, netdata_cpu_idle])) ]
                return (diag_vals, diag_msg, diag_level)

            cores_loaded = 0
            for i_cpu in range(num_cores):

                cpu_name = 'Core %d' % (i_cpu)
                idle = 100 - np.mean(netdata_cpu_idle[i_cpu]['C0 (active)'])
                user = np.mean(netdata_cpu_util[i_cpu]['user'])
                nice = np.mean(netdata_cpu_util[i_cpu]['nice'])
                system = np.mean(netdata_cpu_util[i_cpu]['system'])

                core_level = DiagnosticStatus.OK
                usage = float(user) + float(nice)
                if usage > self._core_load_warn:
                    cores_loaded += 1
                    core_level = DiagnosticStatus.WARN
                if usage > self._core_load_error:
                    core_level = DiagnosticStatus.ERROR

                diag_vals.append(KeyValue(key = 'CPU %s Status' % cpu_name, value = load_dict[core_level]))
                diag_vals.append(KeyValue(key = 'CPU %s User' % cpu_name, value = str(user)))
                diag_vals.append(KeyValue(key = 'CPU %s Nice' % cpu_name, value = str(nice)))
                diag_vals.append(KeyValue(key = 'CPU %s System' % cpu_name, value = str(system)))
                diag_vals.append(KeyValue(key = 'CPU %s Idle' % cpu_name, value = str(idle)))

            # Warn for high load only if we have <= 2 cores that aren't loaded
            if num_cores - cores_loaded <= 2 and num_cores > 2:
                diag_level = DiagnosticStatus.WARN

            diag_msg = load_dict[diag_level]

        except Exception as e:
            diag_level = DiagnosticStatus.ERROR
            diag_msg = 'CPU Usage Exception'
            diag_vals = [ KeyValue(key = 'Exception', value = str(e)), KeyValue(key = 'Traceback', value = str(traceback.format_exc())) ]

        return diag_vals, diag_msg, diag_level


    def check_core_throttling(self, interval=1):
        throt_dict = {DiagnosticStatus.OK: 'OK', DiagnosticStatus.WARN: 'High Thermal Throttling Events',
                      DiagnosticStatus.ERROR: 'No valid Data from NetData'}

        throt_level = DiagnosticStatus.OK

        vals = []

        netdata = query_netdata('cpu.core_throttling', interval)

        for i in range(self._num_cores):
            lbl = 'CPU %d Thermal Throttling Events' % i
            netdata_key = 'cpu%d' % i

            core_mean = 'N/A'
            if netdata is not None and netdata_key in netdata:
                core_data = netdata[netdata_key]
                if core_data is not None:
                    core_mean = np.mean(core_data)

                    if core_mean > self._thermal_throttling_threshold:
                        throt_level = DiagnosticStatus.WARN
            else:
                throt_level = DiagnosticStatus.ERROR

            vals.append(KeyValue(key=lbl, value='%r' % core_mean))

        vals.insert(0, KeyValue(key='Thermal Throttling Status', value=throt_dict[throt_level]))
        vals.append(KeyValue(key='Thermal Throttling Threshold', value=str(self._thermal_throttling_threshold)))

        return throt_level, throt_dict[throt_level], vals


    def check_idlejitter(self, interval=1):
        jitter_dict = {DiagnosticStatus.OK: 'OK', DiagnosticStatus.WARN: 'High IDLE Jitter',
                       DiagnosticStatus.ERROR: 'No valid Data from NetData'}

        jitter_level = DiagnosticStatus.OK

        vals = []

        netdata = query_netdata('system.idlejitter', interval)

        metric_list = [
            ('IDLE Jitter Min', 'min', self._idlejitter_min_threshold, np.min),
            ('IDLE Jitter Max', 'max', self._idlejitter_max_threshold, np.max),
            ('IDLE Jitter Average', 'average', self._idlejitter_average_threshold, np.mean),
        ]

        for metric_label, metric_key, metric_threshold, aggregate_fnc in metric_list:
            metric_aggreagte = 'N/A'
            if netdata is not None and metric_key in netdata:
                metric_data = netdata[metric_key]
                if metric_data is not None:
                    metric_aggreagte = aggregate_fnc(metric_data)

                    if metric_aggreagte > metric_threshold:
                        jitter_level = DiagnosticStatus.WARN
            else:
                jitter_level = DiagnosticStatus.ERROR

            vals.append(KeyValue(key=metric_label, value=str(metric_aggreagte)))
            vals.append(KeyValue(key=metric_label + ' Threshold', value=str(metric_threshold)))

        vals.insert(0, KeyValue(key='IDLE Jitter Status', value=jitter_dict[jitter_level]))

        return jitter_level, jitter_dict[jitter_level], vals


    def check_info(self, event):
        diag_vals = []
        diag_msgs = []
        diag_level = DiagnosticStatus.OK

        if self._check_ipmi:
            ipmi_vals, ipmi_msgs, ipmi_level = self.check_ipmi()
            diag_vals.extend(ipmi_vals)
            diag_msgs.extend(ipmi_msgs)
            diag_level = max(diag_level, ipmi_level)

        if self._check_core_temps:
            interval = math.ceil(self._usage_timer._period.to_sec())
            core_vals, core_msgs, core_level = self.check_core_temps(interval=interval)
            diag_vals.extend(core_vals)
            diag_msgs.extend(core_msgs)
            diag_level = max(diag_level, core_level)

        clock_vals, clock_msgs, clock_level = self.check_clock_speed()
        diag_vals.extend(clock_vals)
        diag_msgs.extend(clock_msgs)
        diag_level = max(diag_level, clock_level)

        diag_log = set(diag_msgs)
        if len(diag_log) > DiagnosticStatus.OK:
            message = ', '.join(diag_log)
        else:
            message = stat_dict[diag_level]

        self._info_stat.values = diag_vals
        self._info_stat.message = message
        self._info_stat.level = diag_level

    def check_usage(self, event):
        diag_vals = []
        diag_msgs = []
        diag_level = DiagnosticStatus.OK

        interval = math.ceil(self._usage_timer._period.to_sec())

        # Check mpstat
        mp_vals, mp_msg, mp_level = self.check_cpu_util(interval=interval)
        diag_vals.extend(mp_vals)
        if mp_level > DiagnosticStatus.OK:
            diag_msgs.append(mp_msg)
        diag_level = max(diag_level, mp_level)

        # Check NetData cpu.core_throttling
        if self._check_thermal_throttling_events:
            throt_level, throt_msg, throt_vals = self.check_core_throttling(interval=interval)
            diag_vals.extend(throt_vals)
            if throt_level > 0:
                diag_msgs.append(throt_msg)
            diag_level = max(diag_level, throt_level)

        # Check NetData system.idlejitter
        if self._check_idlejitter:
            jitter_level, jitter_msg, jitter_vals = self.check_idlejitter(interval=interval)
            diag_vals.extend(jitter_vals)
            if jitter_level > 0:
                diag_msgs.append(jitter_msg)
            diag_level = max(diag_level, jitter_level)

        # Check uptime
        up_vals, up_msg, up_level = self.check_uptime(interval=interval)
        diag_vals.extend(up_vals)
        if up_level > DiagnosticStatus.OK:
            diag_msgs.append(up_msg)
        diag_level = max(diag_level, up_level)

        if diag_msgs and diag_level > DiagnosticStatus.OK:
            usage_msg = ', '.join(set(diag_msgs))
        else:
            usage_msg = stat_dict[diag_level]

        self._usage_stat.values = diag_vals
        self._usage_stat.message = usage_msg
        self._usage_stat.level = diag_level

    def check_memory(self, event):
        diag_vals = []
        diag_msgs = []
        diag_level = DiagnosticStatus.OK

        # Check memory
        interval = math.ceil(self._memory_timer._period.to_sec())
        mem_vals, mem_msg, mem_level = self.check_free_memory(interval=interval)
        diag_vals.extend(mem_vals)
        if mem_level > DiagnosticStatus.OK:
            diag_msgs.append(mem_msg)
        diag_level = max(diag_level, mem_level)

        if diag_msgs and diag_level > DiagnosticStatus.OK:
            memory_msg = ', '.join(set(diag_msgs))
        else:
            memory_msg = stat_dict[diag_level]

        self._memory_stat.values = diag_vals
        self._memory_stat.message = memory_msg
        self._memory_stat.level = diag_level

    def publish_stats(self, event):
        msg = DiagnosticArray()
        msg.header.stamp = rospy.get_rostime()
        msg.status.append(self._info_stat)
        msg.status.append(self._usage_stat)
        msg.status.append(self._memory_stat)
        self._diag_pub.publish(msg)


if __name__ == '__main__':
    hostname = socket.gethostname()

    import optparse
    parser = optparse.OptionParser(usage="usage: cpu_monitor.py [--diag-hostname=cX]")
    parser.add_option("--diag-hostname", dest="diag_hostname",
                      help="Computer name in diagnostics output (ex: 'c1')",
                      metavar="DIAG_HOSTNAME",
                      action="store", default = hostname)
    options, args = parser.parse_args(rospy.myargv())

    try:
        rospy.init_node('cpu_monitor_%s' % hostname)
    except rospy.exceptions.ROSInitException:
        print('CPU monitor is unable to initialize node. Master may not be running.')
        sys.exit(0)

    cpu_node = CPUMonitor(hostname, options.diag_hostname)
    rospy.spin()
