from python_qt_binding.QtCore import QSize

from rqt_robot_dashboard.widgets import IconToolButton


class COBRunstops(IconToolButton):
    """
    Dashboard widget to display Care-O-bot Runstop state.
    """
    def __init__(self, context):
        """
        :param context: the plugin context
        :type context: qt_gui.plugin.Plugin
        """
        
        ok_icon = ['bg-green.svg', 'ic-runstop-off.svg']
        button_engaged_icon = ['bg-red.svg', 'ic-runstop-on.svg']
        scanner_engaged_icon = ['bg-red.svg', 'ic-wireless-runstop-on.svg']
        stale_icon = ['bg-grey.svg', 'ic-runstop-off.svg', 'ol-stale-badge.svg']

        icons = [ok_icon, button_engaged_icon, scanner_engaged_icon, stale_icon]
        super(COBRunstops, self).__init__('Runstop', icons, icons)
        self.setToolTip('Runstop')
        self.set_stale()
        self.setFixedSize(self._icons[0].actualSize(QSize(50, 30)))

    def set_ok(self):
        self.update_state(0)

    def set_button_engaged(self):
        self.update_state(1)

    def set_scanner_engaged(self):
        self.update_state(2)

    def set_stale(self):
        self.update_state(3)