# pylint: skip-file
import json
import struct
import threading
import enum
import weakref
import riconnect

__version__ = "1.8"

class ServiceSignalConnection(object):
    def __init__(self, signal, fn):
        self._signal = signal
        try:
            self._fn = weakref.WeakMethod(fn)
        except TypeError:
            self._fn = fn
        self._connected = True
    def disconnect(self):
        """Disconnect from the signal."""
        if self._connected:
            self._signal._disconnect(self)
            self._connected = False
    def connected(self):
        """Return true if the signal is still connected (disconnect() was not called)."""
        return self._connected

class ServiceSignal(object):
    def __init__(self, svc, signalName, decoderFn=None):
        self._svc = svc
        self._signalName = signalName
        self._connections = []
        self._lock = threading.Lock()
        self._decoderFn = decoderFn
    def connect(self, fn):
        """Connect the specified callback function to the signal."""
        sc = ServiceSignalConnection(self, fn)
        with self._lock:
            if len(self._connections) == 0:
                self._svc.subscribe(self._signalName, self._onSignalReceived)
            self._connections.append(sc)
        return sc
    def _disconnect(self, sc):
        with self._lock:
            if sc in self._connections:
                self._connections.remove(sc)
                sc._connected = False
                if len(self._connections) == 0:
                    self._svc.unsubscribe(self._signalName, self._onSignalReceived)
    def _onSignalReceived(self, payload):
        arg = self._decoderFn(payload) if self._decoderFn else None
        with self._lock:
            try:
                for sc in self._connections:
                    fn = sc._fn() if isinstance(sc._fn, weakref.WeakMethod) else sc._fn
                    if fn is not None:
                        if self._decoderFn is not None:
                            fn(arg)
                        else:
                            fn()
            except Exception as err:
                print(err)


class PowerSupplyConfig(dict):
    def __init__(self, *args, **kwargs):
        self['name'] = None
        self['undervoltage_power_off'] = None
        self['undervoltage_power_on'] = None
        self['voltage_warning_low'] = None
        self['voltage_warning_high'] = None
        self['overvoltage_power_on'] = None
        self['overvoltage_power_off'] = None
        self['description'] = None
        self['needsLicense'] = None
        super().__init__(*args, **kwargs)
    @property
    def name(self):
        return self['name']
    @name.setter
    def name(self, value):
        self['name'] = value
    @property
    def undervoltage_power_off(self):
        return self['undervoltage_power_off']
    @undervoltage_power_off.setter
    def undervoltage_power_off(self, value):
        self['undervoltage_power_off'] = value
    @property
    def undervoltage_power_on(self):
        return self['undervoltage_power_on']
    @undervoltage_power_on.setter
    def undervoltage_power_on(self, value):
        self['undervoltage_power_on'] = value
    @property
    def voltage_warning_low(self):
        return self['voltage_warning_low']
    @voltage_warning_low.setter
    def voltage_warning_low(self, value):
        self['voltage_warning_low'] = value
    @property
    def voltage_warning_high(self):
        return self['voltage_warning_high']
    @voltage_warning_high.setter
    def voltage_warning_high(self, value):
        self['voltage_warning_high'] = value
    @property
    def overvoltage_power_on(self):
        return self['overvoltage_power_on']
    @overvoltage_power_on.setter
    def overvoltage_power_on(self, value):
        self['overvoltage_power_on'] = value
    @property
    def overvoltage_power_off(self):
        return self['overvoltage_power_off']
    @overvoltage_power_off.setter
    def overvoltage_power_off(self, value):
        self['overvoltage_power_off'] = value
    @property
    def description(self):
        return self['description']
    @description.setter
    def description(self, value):
        self['description'] = value
    @property
    def needsLicense(self):
        return self['needsLicense']
    @needsLicense.setter
    def needsLicense(self, value):
        self['needsLicense'] = value

class PowerSupplyConfigChangedPayload(dict):
    def __init__(self, *args, **kwargs):
        self['powerSupply'] = None
        self['config'] = None
        super().__init__(*args, **kwargs)
        if type(self.get('config')) == dict: self['config'] = PowerSupplyConfig(self.get('config'))
    @property
    def powerSupply(self):
        return self['powerSupply']
    @powerSupply.setter
    def powerSupply(self, value):
        self['powerSupply'] = value
    @property
    def config(self):
        return self['config']
    @config.setter
    def config(self, value):
        self['config'] = value

class PowerSupplyRestoreStatus(dict):
    def __init__(self, *args, **kwargs):
        self['restored'] = None
        self['restore_failed_power1'] = None
        self['error_code_power1'] = None
        self['error_string_power1'] = None
        self['config_name_power1'] = None
        self['restore_failed_power2'] = None
        self['error_code_power2'] = None
        self['error_string_power2'] = None
        self['config_name_power2'] = None
        super().__init__(*args, **kwargs)
    @property
    def restored(self):
        return self['restored']
    @restored.setter
    def restored(self, value):
        self['restored'] = value
    @property
    def restore_failed_power1(self):
        return self['restore_failed_power1']
    @restore_failed_power1.setter
    def restore_failed_power1(self, value):
        self['restore_failed_power1'] = value
    @property
    def error_code_power1(self):
        return self['error_code_power1']
    @error_code_power1.setter
    def error_code_power1(self, value):
        self['error_code_power1'] = value
    @property
    def error_string_power1(self):
        return self['error_string_power1']
    @error_string_power1.setter
    def error_string_power1(self, value):
        self['error_string_power1'] = value
    @property
    def config_name_power1(self):
        return self['config_name_power1']
    @config_name_power1.setter
    def config_name_power1(self, value):
        self['config_name_power1'] = value
    @property
    def restore_failed_power2(self):
        return self['restore_failed_power2']
    @restore_failed_power2.setter
    def restore_failed_power2(self, value):
        self['restore_failed_power2'] = value
    @property
    def error_code_power2(self):
        return self['error_code_power2']
    @error_code_power2.setter
    def error_code_power2(self, value):
        self['error_code_power2'] = value
    @property
    def error_string_power2(self):
        return self['error_string_power2']
    @error_string_power2.setter
    def error_string_power2(self, value):
        self['error_string_power2'] = value
    @property
    def config_name_power2(self):
        return self['config_name_power2']
    @config_name_power2.setter
    def config_name_power2(self, value):
        self['config_name_power2'] = value

class LicenseData(dict):
    def __init__(self, *args, **kwargs):
        self['name'] = None
        self['description'] = None
        self['data'] = None
        self['is_valid'] = None
        self['is_trial'] = None
        self['seconds_till_expire'] = None
        super().__init__(*args, **kwargs)
    @property
    def name(self):
        return self['name']
    @name.setter
    def name(self, value):
        self['name'] = value
    @property
    def description(self):
        return self['description']
    @description.setter
    def description(self, value):
        self['description'] = value
    @property
    def data(self):
        return self['data']
    @data.setter
    def data(self, value):
        self['data'] = value
    @property
    def is_valid(self):
        return self['is_valid']
    @is_valid.setter
    def is_valid(self, value):
        self['is_valid'] = value
    @property
    def is_trial(self):
        return self['is_trial']
    @is_trial.setter
    def is_trial(self, value):
        self['is_trial'] = value
    @property
    def seconds_till_expire(self):
        return self['seconds_till_expire']
    @seconds_till_expire.setter
    def seconds_till_expire(self, value):
        self['seconds_till_expire'] = value

def _deviceservice_error_decoder(payload):
    return json.loads(payload.decode())
def _deviceservice_displaybrightnesschanged_decoder(payload):
    return struct.unpack("!I", payload)[0]
def _deviceservice_displayautodimmingchanged_decoder(payload):
    return struct.unpack("!?", payload)[0]
def _deviceservice_displaystandbytimeoutchanged_decoder(payload):
    return struct.unpack("!i", payload)[0]
def _deviceservice_soundvolumechanged_decoder(payload):
    return struct.unpack("!I", payload)[0]
def _deviceservice_powersupplyconfigurationchanged_decoder(payload):
    return PowerSupplyConfigChangedPayload(json.loads(payload.decode()))
def _deviceservice_powersupplyconfigsynchronizationchanged_decoder(payload):
    return struct.unpack("!?", payload)[0]
def _deviceservice_displaytouchenabledchanged_decoder(payload):
    return struct.unpack("!?", payload)[0]

class DeviceService(object):
    """Brief service description.
       
       Detailed service description."""

    class Error(enum.IntEnum):
        ERROR_NONE = 0
        ERROR_NO_LICENSE = 12000
        ERROR_POWER_CONFIG_NOT_FOUND = 12100
        ERROR_POWER_VOLTAGE_OUT_OF_RANGE = 12101
        ERROR_INVALID_POWER_SUPPLY = 12102

    class PowerSupply(enum.IntEnum):
        POWER1 = 1
        POWER2 = 2

    ERROR_NONE = Error.ERROR_NONE
    ERROR_NO_LICENSE = Error.ERROR_NO_LICENSE
    ERROR_POWER_CONFIG_NOT_FOUND = Error.ERROR_POWER_CONFIG_NOT_FOUND
    ERROR_POWER_VOLTAGE_OUT_OF_RANGE = Error.ERROR_POWER_VOLTAGE_OUT_OF_RANGE
    ERROR_INVALID_POWER_SUPPLY = Error.ERROR_INVALID_POWER_SUPPLY
    POWER1 = PowerSupply.POWER1
    POWER2 = PowerSupply.POWER2

    def __init__(self, address):
        self._svc = riconnect.Service("DeviceService")
        self._svc.open(address)

        # service signals
        self._error = ServiceSignal(self._svc, "error", decoderFn=_deviceservice_error_decoder)
        self._displayBrightnessChanged = ServiceSignal(self._svc, "displayBrightnessChanged", decoderFn=_deviceservice_displaybrightnesschanged_decoder)
        self._displayAutoDimmingChanged = ServiceSignal(self._svc, "displayAutoDimmingChanged", decoderFn=_deviceservice_displayautodimmingchanged_decoder)
        self._displayStandbyTimeoutChanged = ServiceSignal(self._svc, "displayStandbyTimeoutChanged", decoderFn=_deviceservice_displaystandbytimeoutchanged_decoder)
        self._soundVolumeChanged = ServiceSignal(self._svc, "soundVolumeChanged", decoderFn=_deviceservice_soundvolumechanged_decoder)
        self._licenseUpdate = ServiceSignal(self._svc, "licenseUpdate")
        self._powerSupplyConfigurationChanged = ServiceSignal(self._svc, "powerSupplyConfigurationChanged", decoderFn=_deviceservice_powersupplyconfigurationchanged_decoder)
        self._powerSupplyConfigSynchronizationChanged = ServiceSignal(self._svc, "powerSupplyConfigSynchronizationChanged", decoderFn=_deviceservice_powersupplyconfigsynchronizationchanged_decoder)
        self._displayTouchEnabledChanged = ServiceSignal(self._svc, "displayTouchEnabledChanged", decoderFn=_deviceservice_displaytouchenabledchanged_decoder)
        self._lock = threading.Lock()

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self._svc.close()

    def __del__(self):
        self._svc.close()

    def displayBrightness(self):
        """Return the display brightness of the GUI.
           
           Returns: int"""
        inputs = None
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("displayBrightness_629536f9", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        outputs = []

        outputs.append(rvalues[0].u32)
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def setDisplayBrightness(self, value):
        """Set the GUI display brightness.
           
           Arguments:
             value (int): the new brightness (range 0 to 100)"""
        inputs = [riconnect.Value() for i in range(0, 1)]
        inputs[0].u32 = value
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("setDisplayBrightness_a4137c6a", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        
    def displayAutoDimming(self):
        """Return true if display brightness auto dimming is enabled.
           
           Returns: bool"""
        inputs = None
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("displayAutoDimming_22e2b25f", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        outputs = []

        outputs.append(True if rvalues[0].i32 > 0 else False)
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def setDisplayAutoDimming(self, enable):
        """Enable/disable display brightness auto dimming.
           
           Arguments:
             enable (bool): """
        inputs = [riconnect.Value() for i in range(0, 1)]
        inputs[0].i32 = 1 if enable else 0
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("setDisplayAutoDimming_9cda73fa", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        
    def displayStandbyTimeout(self):
        """Return the display standby timeout in seconds.
           
           Returns: int"""
        inputs = None
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("displayStandbyTimeout_880bd63b", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        outputs = []

        outputs.append(rvalues[0].i32)
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def setDisplayStandbyTimeout(self, timeout):
        """Set display standby timeout.
           
           A value smaller than five will disable the automatic display standby mode.
           
           Arguments:
             timeout (int): new timeout in seconds"""
        inputs = [riconnect.Value() for i in range(0, 1)]
        inputs[0].i32 = timeout
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("setDisplayStandbyTimeout_4bf4ffb3", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        
    def soundVolume(self):
        """Return the sound volume of the integrated speaker.
           
           Returns: int"""
        inputs = None
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("soundVolume_f79ac18e", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        outputs = []

        outputs.append(rvalues[0].u32)
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def setSoundVolume(self, value):
        """Set the sound volume of the integrated speaker.
           
           Arguments:
             value (int): the new sound volume (range 0 to 100)"""
        inputs = [riconnect.Value() for i in range(0, 1)]
        inputs[0].u32 = value
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("setSoundVolume_8cc1bb2e", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        
    def syncTime(self):
        """Synchronize system time."""
        inputs = None
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("syncTime_7dd4d5b5", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0, timeout=None)
        
    def setTimeZone(self, value):
        """Set the current time zone.
           
           Arguments:
             value (str): the new time zone e.g. "Europe/Vienna" """
        inputs = [riconnect.Value() for i in range(0, 1)]
        inputs[0].s = value
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("setTimeZone_2ddbb14a", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        
    def registerLicense(self, name, description):
        """Register a software feature which requires a license for activation.
           
           Arguments:
             name (str): the name of the licensed software feature
             description (str): a detailed description of the licensed software feature"""
        inputs = [riconnect.Value() for i in range(0, 2)]
        inputs[0].s = name
        inputs[1].s = description
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("registerLicense_eddd42d7", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        
    def getLicenses(self):
        """Return information about available licenses.
           
           The returned license information contain the name and description of the
           licensed software feature, the license state, and in case of a trial
           license, the seconds until it expires.
           
           Returns: list(LicenseData)"""
        inputs = None
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("getLicenses_d3ac0603", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        outputs = []

        outputs.append([LicenseData(k) for k in json.loads(rvalues[0].s)])
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def licenseIsValid(self, name):
        """Check if a valid license is available for a particular software feature.
           
           Returns true if a valid license for the specified feature is available.
           
           Arguments:
             name (str): the name of the licensed software feature
           
           Returns: bool"""
        inputs = [riconnect.Value() for i in range(0, 1)]
        inputs[0].s = name
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("licenseIsValid_89cbaf5e", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        outputs = []

        outputs.append(True if rvalues[0].i32 > 0 else False)
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def updateLicenses(self):
        """Search for license files in the license folders, and update the state (expiration time) of the trial licenses."""
        inputs = None
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("updateLicenses_b0eb3005", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        
    def powerSupplyConfigurations(self):
        """Return a list of available power supply configurations.
           
           Returns: list(PowerSupplyConfig)"""
        inputs = None
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("powerSupplyConfigurations_a5e661f7", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        outputs = []

        outputs.append([PowerSupplyConfig(k) for k in json.loads(rvalues[0].s)])
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def powerSupplyConfiguration(self, ps):
        """Get the power supply configuration for the specified power supply.
           
           Arguments:
             ps (PowerSupply): 
           
           Returns: PowerSupplyConfig"""
        inputs = [riconnect.Value() for i in range(0, 1)]
        inputs[0].i32 = int(ps)
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("powerSupplyConfiguration_6f081bdf", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        outputs = []

        outputs.append(PowerSupplyConfig(json.loads(rvalues[0].s)))
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def setPowerSupplyConfiguration(self, ps, configName):
        """Set the power supply configuration for the specified power supply.
           
           Arguments:
             ps (PowerSupply): 
             configName (str): """
        inputs = [riconnect.Value() for i in range(0, 2)]
        inputs[0].i32 = int(ps)
        inputs[1].s = configName
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("setPowerSupplyConfiguration_038b3f47", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        
    def powerSupplyRestoreStatus(self):
        """Return the status of the last time the power supply configuration were restored (e.g. during service startup).
           
           Returns: PowerSupplyRestoreStatus"""
        inputs = None
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("powerSupplyRestoreStatus_bdbc3087", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        outputs = []

        outputs.append(PowerSupplyRestoreStatus(json.loads(rvalues[0].s)))
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def isPowerSupplyConfigSynchronizationEnabled(self):
        """Return true if power supply configuration synchronization is enabled.
           
           Returns: bool"""
        inputs = None
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("isPowerSupplyConfigSynchronizationEnabled_f3de3860", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        outputs = []

        outputs.append(True if rvalues[0].i32 > 0 else False)
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def setPowerSupplyConfigSynchronizationEnabled(self, value):
        """Enable/Disable power supply configuration synchronization.
           
           If enabled then setting the power supply configuration of one power supply
           will also set the same configuration for the other power supply.
           
           Arguments:
             value (bool): """
        inputs = [riconnect.Value() for i in range(0, 1)]
        inputs[0].i32 = 1 if value else 0
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("setPowerSupplyConfigSynchronizationEnabled_b07720ce", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        
    def getAtmosTemperature(self):
        """Returns atmospheric temperature.
           
           This returns the atmospheric temperature in celsius degree if sensor is available, else nan.
           
           Returns: float"""
        inputs = None
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("getAtmosTemperature_d13e60fc", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        outputs = []

        outputs.append(rvalues[0].f)
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def getAtmosRelHumidity(self):
        """Returns atmospheric humidity.
           
           This returns the relative atmospheric humidity in percent if sensor is available, else nan.
           
           Returns: float"""
        inputs = None
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("getAtmosRelHumidity_37225b31", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        outputs = []

        outputs.append(rvalues[0].f)
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def getAtmosPressure(self):
        """Returns atmospheric pressure.
           
           This returns the atmospheric pressure in mbar if sensor is available, else nan.
           
           Returns: float"""
        inputs = None
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("getAtmosPressure_294bcc6b", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        outputs = []

        outputs.append(rvalues[0].f)
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def enableAtmosPressureAveraging(self, enable):
        """Activate background thread for averaging of atmospheric pressure sensor values.
           
           Arguments:
             enable (bool): Start (1) or stop (0) averaging thread"""
        inputs = [riconnect.Value() for i in range(0, 1)]
        inputs[0].i32 = 1 if enable else 0
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("enableAtmosPressureAveraging_1bf7df4e", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        
    def getAtmosPressureAverage(self):
        """Returns averaged atmospheric pressure value.
           
           This returns the averaged atmospheric pressure in mbar since start of averaging or last reading, else nan.
           
           Returns: float"""
        inputs = None
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("getAtmosPressureAverage_865d523d", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        outputs = []

        outputs.append(rvalues[0].f)
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def displayTouchEnabled(self):
        """Return true if the touch input of the display is enabled.
           
           New in version 1.8.
           
           Returns: bool"""
        inputs = None
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("displayTouchEnabled_d375e626", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        outputs = []

        outputs.append(True if rvalues[0].i32 > 0 else False)
        return outputs[0] if len(outputs) == 1 else tuple(outputs)

    def setDisplayTouchEnabled(self, enabled):
        """Enable/disable display touch input.
           
           New in version 1.8.
           
           Arguments:
             enabled (bool): true to enable touch input"""
        inputs = [riconnect.Value() for i in range(0, 1)]
        inputs[0].i32 = 1 if enabled else 0
        inputTransfers = None
        with self._lock:
            rvalues, rtransfers = self._svc.callFunction("setDisplayTouchEnabled_4749cbb3", inputs=inputs, inputTransfers=inputTransfers, numOutputTransfers=0)
        
    def save(self):
        """Save service properties."""
        with self._lock:
            self._svc.callFunction("__risc__service_properties__save_fn")

    def restore(self):
        """Restore service properties."""
        with self._lock:
            self._svc.callFunction("__risc__service_properties__restore_fn")

    def error(self):
        """The error signal allows services to report problems that are
           unrelated to service calls. It was intended to allow services
           to report hardware related problems. But since most services
           provide some kind of business logic, this signal is rarely used.
           
           Returns: ServiceSignal
           Payload: dict"""
        return self._error

    def displayBrightnessChanged(self):
        """This signal is emitted when the display brightness changed.
           
           Returns: ServiceSignal
           Payload: int"""
        return self._displayBrightnessChanged

    def displayAutoDimmingChanged(self):
        """This signal is emitted when the display brightness auto dimming state changed.
           
           Returns: ServiceSignal
           Payload: bool"""
        return self._displayAutoDimmingChanged

    def displayStandbyTimeoutChanged(self):
        """This signal is emitted when the display standby timeout changed.
           
           Returns: ServiceSignal
           Payload: int"""
        return self._displayStandbyTimeoutChanged

    def soundVolumeChanged(self):
        """This signal is emitted when the sound volume changed.
           
           Returns: ServiceSignal
           Payload: int"""
        return self._soundVolumeChanged

    def licenseUpdate(self):
        """This signal is emitted when the license information changed.
           
           A change of the license information can be caused by the registration of
           a new feature or an update of the exiting licenses.
           
           Returns: ServiceSignal"""
        return self._licenseUpdate

    def powerSupplyConfigurationChanged(self):
        """This signal is emitted when the power supply configuration changed.
           
           Returns: ServiceSignal
           Payload: PowerSupplyConfigChangedPayload"""
        return self._powerSupplyConfigurationChanged

    def powerSupplyConfigSynchronizationChanged(self):
        """This signal is emitted when the power supply configuration synchronization changed.
           
           Returns: ServiceSignal
           Payload: bool"""
        return self._powerSupplyConfigSynchronizationChanged

    def displayTouchEnabledChanged(self):
        """This signal is emitted when the display touch input enabled state changed.
           
           New in version 1.8.
           
           Returns: ServiceSignal
           Payload: bool"""
        return self._displayTouchEnabledChanged
