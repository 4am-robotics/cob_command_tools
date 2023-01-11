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
import socket
import psutil
import numpy as np
import math
import requests

import rospy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

from netdata_interface.netdata_interface import NetdataInterface

stat_dict = { DiagnosticStatus.OK: 'OK', DiagnosticStatus.WARN: 'Warning', DiagnosticStatus.ERROR: 'Error', DiagnosticStatus.STALE: 'Stale' }

class CPUMonitor():
    def __init__(self, hostname, diag_hostname):
        self._netdata_interface = NetdataInterface()

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

    ##\brief Check CPU core temps
    def check_core_temps(self, interval=1):
        diag_vals = []
        diag_msgs = []
        diag_level = DiagnosticStatus.OK

        try:
            # _ vs -
            netdata_module_name_core_temps = ['sensors.coretemp_isa_0000_temperature',
                                              'sensors.coretemp-isa-0000_temperature']
            error_count = 0
            netdata_module_name_err = ''
            for name in netdata_module_name_core_temps:
                try:
                    netdata_core_temp, error = self._netdata_interface.query_netdata(name, interval)

                # count individual connection errors for mutliple chart names (different netdata versions)
                except requests.ConnectionError as err:
                    error_count += 1
                    netdata_module_name_err += name + ' '

                    netdata_core_temp = None
                    error = str(err)


                if netdata_core_temp:
                    break

            netdata_module_name_err = "{} of {} failed: {}".format(error_count, len(netdata_module_name_core_temps), netdata_module_name_err)

            if not netdata_core_temp:
                diag_level = DiagnosticStatus.WARN
                diag_msgs = [ 'Core Temp Error' ]
                diag_vals = [ KeyValue(key = 'Core Temp Error', value = 'Could not fetch data from netdata'),
                              KeyValue(key = 'Failed Chart Names', value = netdata_module_name_err),
                              KeyValue(key = 'Output', value = netdata_core_temp),
                              KeyValue(key = 'Error', value= error) ]
                return (diag_vals, diag_msgs, diag_level)

            del netdata_core_temp['time']
            del netdata_core_temp['Package id 0']

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
            netdata_cpu_freq, error = self._netdata_interface.query_netdata('cpu.cpufreq', interval)
            if not netdata_cpu_freq:
                diag_level = DiagnosticStatus.WARN
                diag_msgs = [ 'Clock Speed Error' ]
                diag_vals = [ KeyValue(key = 'Clock Speed Error', value = 'Could not fetch data from netdata'),
                              KeyValue(key = 'Output', value = netdata_cpu_freq),
                              KeyValue(key = 'Error', value= error) ]
                return (diag_vals, diag_msgs, diag_level)

            del netdata_cpu_freq["time"]

            for cpu_name, values in netdata_cpu_freq.items():
                diag_vals.append(KeyValue(key = 'Core %d (MHz)' % int(cpu_name[-1]), value = str(np.mean(values))))

            # get max freq
            netdata_info = self._netdata_interface.query_netdata_info()
            if not netdata_info:
                diag_level = DiagnosticStatus.WARN
                diag_msgs = [ 'Clock Speed Error' ]
                diag_vals = [ KeyValue(key = 'Clock Speed Error', value = 'Could not fetch data from netdata'),
                              KeyValue(key = 'Output', value = netdata_info) ]
                return (diag_vals, diag_msgs, diag_level)

            max_cpu_freq = float(netdata_info['cpu_freq'])/1e6
            diag_vals.append(KeyValue(key = 'Maximum Frequency (MHz)', value = str(max_cpu_freq)))

        except requests.ConnectionError as e:
            diag_level = DiagnosticStatus.ERROR
            diag_msgs = [ 'Clock Speed Connection Error' ]
            diag_vals = [ KeyValue(key = 'Exception', value = str(e)), KeyValue(key = 'Traceback', value = str(traceback.format_exc())) ]

        except Exception as e:
            diag_level = DiagnosticStatus.ERROR
            diag_msgs = [ 'Clock Speed Exception' ]
            diag_vals = [ KeyValue(key = 'Exception', value = str(e)), KeyValue(key = 'Traceback', value = str(traceback.format_exc())) ]

        return diag_vals, diag_msgs, diag_level

    ##\brief Uses 'uptime' to see system uptime
    def check_uptime(self, interval=1):
        diag_vals = []
        diag_msg = ''
        diag_level = DiagnosticStatus.OK

        try:
            netdata_uptime, error = self._netdata_interface.query_netdata('system.uptime', interval)
            if not netdata_uptime:
                diag_level = DiagnosticStatus.WARN
                diag_msg = 'Uptime Error'
                diag_vals = [ KeyValue(key = 'Uptime Error', value = 'Could not fetch data from netdata'),
                              KeyValue(key = 'Output', value = netdata_uptime),
                              KeyValue(key = 'Error', value= error) ]
                return (diag_vals, diag_msg, diag_level)

            del netdata_uptime['time']

            diag_vals.append(KeyValue(key = 'Uptime', value = str(np.max(netdata_uptime['uptime'].astype(float)))))

        except requests.ConnectionError as e:
            diag_level = DiagnosticStatus.ERROR
            diag_msg = 'Uptime Connection Error'
            diag_vals = [ KeyValue(key = 'Exception', value = str(e)), KeyValue(key = 'Traceback', value = str(traceback.format_exc())) ]

        except Exception as e:
            diag_level = DiagnosticStatus.ERROR
            diag_msg = 'Uptime Exception'
            diag_vals = [ KeyValue(key = 'Exception', value = str(e)), KeyValue(key = 'Traceback', value = str(traceback.format_exc())) ]

        return diag_vals, diag_msg, diag_level

    ##\brief Uses 'system.load' to see load average
    def check_load(self, interval=1):
        diag_vals = []
        diag_msg = ''
        diag_level = DiagnosticStatus.OK

        load_dict = { DiagnosticStatus.OK: 'OK', DiagnosticStatus.WARN: 'High Load', DiagnosticStatus.ERROR: 'Very High Load' }

        try:
            netdata_cpu_load, error = self._netdata_interface.query_netdata('system.load', interval)
            if not netdata_cpu_load:
                diag_level = DiagnosticStatus.WARN
                diag_msg = 'Load Error'
                diag_vals = [ KeyValue(key = 'Load Error', value = 'Could not fetch data from netdata'),
                              KeyValue(key = 'Output', value = netdata_cpu_load),
                              KeyValue(key = 'Error', value= error) ]
                return (diag_vals, diag_msg, diag_level)

            del netdata_cpu_load['time']

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

        except requests.ConnectionError as e:
            diag_level = DiagnosticStatus.ERROR
            diag_msg = 'Load Connection Error'
            diag_vals = [ KeyValue(key = 'Exception', value = str(e)), KeyValue(key = 'Traceback', value = str(traceback.format_exc())) ]

        except Exception as e:
            diag_level = DiagnosticStatus.ERROR
            diag_msg = 'Load Exception'
            diag_vals = [ KeyValue(key = 'Exception', value = str(e)), KeyValue(key = 'Traceback', value = str(traceback.format_exc())) ]

        return diag_vals, diag_msg, diag_level


    ##\brief Uses 'free -m' to check free memory
    def check_free_memory(self, interval=1):
        diag_vals = []
        diag_msg = ''
        diag_level = DiagnosticStatus.OK

        mem_dict = { DiagnosticStatus.OK: 'OK', DiagnosticStatus.WARN: 'Low Memory', DiagnosticStatus.ERROR: 'Very Low Memory' }

        try:
            netdata_mem, error = self._netdata_interface.query_netdata('system.ram', interval)
            if not netdata_mem:
                diag_level = DiagnosticStatus.WARN
                diag_msg = 'Memory Usage Error'
                diag_vals = [ KeyValue(key = 'Memory Usage Error', value = 'Could not fetch data from netdata'),
                              KeyValue(key = 'Output', value = netdata_mem),
                              KeyValue(key = 'Error', value= error) ]
                return (diag_vals, diag_msg, diag_level)

            del netdata_mem['time']

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

            netdata_swp, error = self._netdata_interface.query_netdata('system.swap', interval)
            if not netdata_swp:
                diag_level = DiagnosticStatus.WARN
                diag_msg = 'Swap Usage Error'
                diag_vals = [ KeyValue(key = 'Swap Usage Error', value = 'Could not fetch data from netdata'),
                              KeyValue(key = 'Output', value = netdata_swp),
                              KeyValue(key = 'Error', value= error) ]
                return (diag_vals, diag_msg, diag_level)

            del netdata_swp['time']

            # Swap
            swap_vals = {k: np.mean(v.astype(float)) for k, v in netdata_swp.items()}
            total_swp = sum([val for val in swap_vals.values()])
            free_swp = swap_vals['free']
            used_swp = swap_vals['used']

            diag_vals.append(KeyValue(key = 'Swap Total', value = str(total_swp)))
            diag_vals.append(KeyValue(key = 'Swap Used', value = str(used_swp)))
            diag_vals.append(KeyValue(key = 'Swap Free', value = str(free_swp)))

            diag_msg = mem_dict[diag_level]

        except requests.ConnectionError as e:
            diag_level = DiagnosticStatus.ERROR
            diag_msg = 'Memory Usage Connection Error'
            diag_vals = [ KeyValue(key = 'Exception', value = str(e)), KeyValue(key = 'Traceback', value = str(traceback.format_exc())) ]

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
            netdata_info = self._netdata_interface.query_netdata_info()
            if not netdata_info:
                diag_level = DiagnosticStatus.WARN
                diag_msg = 'CPU Usage Error'
                diag_vals = [ KeyValue(key = 'CPU Usage Error', value = 'Could not fetch data from netdata'),
                              KeyValue(key = 'Output', value = netdata_info) ]
                return (diag_vals, diag_msg, diag_level)

            num_cores = int(netdata_info['cores_total'])
            netdata_system_cpu, error = self._netdata_interface.query_netdata('system.cpu', interval)
            if not netdata_system_cpu:
                diag_level = DiagnosticStatus.WARN
                diag_msg = 'CPU Usage Error'
                diag_vals = [ KeyValue(key = 'CPU Usage Error', value = 'Could not fetch data from netdata'),
                              KeyValue(key = 'Output', value = netdata_system_cpu),
                              KeyValue(key = 'Error', value= error) ]
                return (diag_vals, diag_msg, diag_level)

            netdata_cpu_util = [self._netdata_interface.query_netdata('cpu.cpu%d' % i, interval) for i in range(num_cores)]
            netdata_cpu_idle = [self._netdata_interface.query_netdata('cpu.cpu%d_cpuidle' % i, interval) for i in range(num_cores)]

            if any([data == None for data, error in netdata_cpu_util]):
                diag_level = DiagnosticStatus.ERROR
                diag_msg = 'CPU Util Error'
                diag_vals = [ KeyValue(key = 'CPU Util Error', value = 'Could not fetch data from netdata'),
                              KeyValue(key = 'Output', value = netdata_cpu_util) ]
                return (diag_vals, diag_msg, diag_level)
            if any([data == None for data, error in netdata_cpu_idle]):
                diag_level = DiagnosticStatus.ERROR
                diag_msg = 'CPU Idle Error'
                diag_vals = [ KeyValue(key = 'CPU Idle Error', value = 'Could not fetch data from netdata'),
                              KeyValue(key = 'Output', value = netdata_cpu_idle) ]
                return (diag_vals, diag_msg, diag_level)

            cores_loaded = 0
            for i_cpu in range(num_cores):

                cpu_name = 'Core %d' % (i_cpu)
                idle = 100 - np.mean(netdata_cpu_idle[i_cpu][0]['C0 (active)'])
                user = np.mean(netdata_cpu_util[i_cpu][0]['user'])
                nice = np.mean(netdata_cpu_util[i_cpu][0]['nice'])
                system = np.mean(netdata_cpu_util[i_cpu][0]['system'])

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

        except requests.ConnectionError as e:
            diag_level = DiagnosticStatus.ERROR
            diag_msg = 'CPU Usage Connection Error'
            diag_vals = [ KeyValue(key = 'Exception', value = str(e)), KeyValue(key = 'Traceback', value = str(traceback.format_exc())) ]

        except Exception as e:
            diag_level = DiagnosticStatus.ERROR
            diag_msg = 'CPU Usage Exception'
            diag_vals = [ KeyValue(key = 'Exception', value = str(e)), KeyValue(key = 'Traceback', value = str(traceback.format_exc())) ]

        return diag_vals, diag_msg, diag_level


    def check_core_throttling(self, interval=1):
        throt_dict = {DiagnosticStatus.OK: 'OK', DiagnosticStatus.WARN: 'High Thermal Throttling Events',
                      DiagnosticStatus.ERROR: 'No valid Data from NetData'}

        throt_level = DiagnosticStatus.OK
        throt_msg = ''
        throt_vals = []

        try:
            netdata, error = self._netdata_interface.query_netdata('cpu.core_throttling', interval)
            if not netdata:
                throt_level = DiagnosticStatus.WARN
                throt_msg = 'Core Throttling Error'
                throt_vals = [ KeyValue(key = 'Core Throttling Error', value = 'Could not fetch data from netdata'),
                                KeyValue(key = 'Output', value = netdata),
                                KeyValue(key = 'Error', value= error) ]
                return (throt_vals, throt_msg, throt_level)

            for i in range(self._num_cores):
                lbl = 'CPU %d Thermal Throttling Events' % i
                netdata_key = 'cpu%d' % i

                core_mean = 'N/A'
                if netdata_key in netdata:
                    core_data = netdata[netdata_key]
                    if core_data is not None:
                        core_mean = np.mean(core_data)

                        if core_mean > self._thermal_throttling_threshold:
                            throt_level = DiagnosticStatus.WARN
                else:
                    throt_level = DiagnosticStatus.ERROR

                throt_vals.append(KeyValue(key=lbl, value='%r' % core_mean))

            throt_vals.insert(0, KeyValue(key='Thermal Throttling Status', value=throt_msg))
            throt_vals.append(KeyValue(key='Thermal Throttling Threshold', value=str(self._thermal_throttling_threshold)))

            throt_msg = throt_dict[throt_level]

        except requests.ConnectionError as e:
            throt_level = DiagnosticStatus.ERROR
            throt_msg = 'Thermal Throttling Connection Error'
            throt_vals = [ KeyValue(key = 'Exception', value = str(e)), KeyValue(key = 'Traceback', value = str(traceback.format_exc())) ]

        except Exception as e:
            throt_level = DiagnosticStatus.ERROR
            throt_msg = 'Thermal Throttling Exception'
            throt_vals = [ KeyValue(key = 'Exception', value = str(e)), KeyValue(key = 'Traceback', value = str(traceback.format_exc())) ]

        return throt_vals, throt_msg, throt_level


    def check_idlejitter(self, interval=1):
        jitter_dict = {DiagnosticStatus.OK: 'OK', DiagnosticStatus.WARN: 'High IDLE Jitter',
                       DiagnosticStatus.ERROR: 'No valid Data from NetData'}

        jitter_level = DiagnosticStatus.OK
        jitter_msg = ''
        jitter_vals = []

        try:
            netdata, error = self._netdata_interface.query_netdata('system.idlejitter', interval)
            if not netdata:
                jitter_level = DiagnosticStatus.WARN
                jitter_msg = 'IDLE Jitter Error'
                jitter_vals = [ KeyValue(key = 'Core Throttling Error', value = 'Could not fetch data from netdata'),
                                KeyValue(key = 'Output', value = netdata),
                                KeyValue(key = 'Error', value= error) ]
                return (jitter_vals, jitter_msg, jitter_level)

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

                jitter_vals.append(KeyValue(key=metric_label, value=str(metric_aggreagte)))
                jitter_vals.append(KeyValue(key=metric_label + ' Threshold', value=str(metric_threshold)))

            jitter_vals.insert(0, KeyValue(key='IDLE Jitter Status', value=jitter_msg))
            jitter_msg = jitter_dict[jitter_level]

        except requests.ConnectionError as e:
            jitter_level = DiagnosticStatus.ERROR
            jitter_msg = 'IDLE Jitter Connection Error'
            jitter_vals = [ KeyValue(key = 'Exception', value = str(e)), KeyValue(key = 'Traceback', value = str(traceback.format_exc())) ]

        except Exception as e:
            jitter_level = DiagnosticStatus.ERROR
            jitter_msg = 'IDLE Jitter Exception'
            jitter_vals = [ KeyValue(key = 'Exception', value = str(e)), KeyValue(key = 'Traceback', value = str(traceback.format_exc())) ]

        return jitter_vals, jitter_msg, jitter_level


    def check_info(self, event):
        diag_vals = []
        diag_msgs = []
        diag_level = DiagnosticStatus.OK

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
            throt_vals, throt_msg, throt_level = self.check_core_throttling(interval=interval)
            diag_vals.extend(throt_vals)
            if throt_level > 0:
                diag_msgs.append(throt_msg)
            diag_level = max(diag_level, throt_level)

        # Check NetData system.idlejitter
        if self._check_idlejitter:
            jitter_vals, jitter_msg, jitter_level = self.check_idlejitter(interval=interval)
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

        # Check load
        load_vals, load_msg, load_level = self.check_load(interval=interval)
        diag_vals.extend(load_vals)
        if load_level > DiagnosticStatus.OK:
            diag_msgs.append(load_msg)
        diag_level = max(diag_level, load_level)

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
