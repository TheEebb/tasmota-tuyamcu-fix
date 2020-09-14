/*
  xdrv_23_zigbee_5__constants.ino - zigbee support for Tasmota

  Copyright (C) 2020  Theo Arends and Stephan Hadinger

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifdef USE_ZIGBEE

// Below is a compilation of Strings used in Zigbee commands and converters.
// Instead of using pointer to strings (4 bytes), we are using an offset (16 bits)
// into an array of strings - which leads to a 1/3 more compact structure.

// To generate the code below use https://tasmota.hadinger.fr/util
// and copy/paste the entire arrays `Z_PostProcess` and `Z_Commands` concatenated
// Note: the 'C' syntax is irrelevant, the parser only looks for `Z_(<string>)`

// In addition the Python3 code used is below:

/*Python code to generate code below

import re
pat = r"Z\(([^\)]+)\)"      # extract text in Z() macro

def clean(s):
    return s.strip(" \t\n\r")

def strings_to_pmem(arg):
    #strings = arg.split("\n")
    strings = re.findall(pat, arg)
    
    # do some basic cleaning
    strings_cleaned = [ clean(x) for x in strings if clean(x) != ""]
    
    # remove duplicates
    strings_cleaned = list(dict.fromkeys(strings_cleaned))
    
    out_s = "const char Z_strings[] PROGMEM = \n"
    out_i = "enum Z_offsets {\n"
    
    index = 0;
    # add a first empty string
    out_s += "  \"\\x00\"\n"
    out_i +=  "  Zo_ = " + str(index) + ",\n"
    index += 1
    
    for s in strings_cleaned:
        out_s += "  \"" + s + "\" \"\\x00\"\n"
        out_i += "  Zo_" + s + " = " + str(index) + ",\n"
        index += len(s) + 1 # add one for null char
    
    out_s += "  \"\\x00\";"
    out_i += "};"
    
    return ("", out_s, out_i)


*/

/*
  DO NOT EDIT
*/

const char Z_strings[] PROGMEM = 
  "\x00"
  "ZCLVersion" "\x00"
  "AppVersion" "\x00"
  "StackVersion" "\x00"
  "HWVersion" "\x00"
  "Manufacturer" "\x00"
  "ModelId" "\x00"
  "DateCode" "\x00"
  "PowerSource" "\x00"
  "GenericDeviceClass" "\x00"
  "GenericDeviceType" "\x00"
  "ProductCode" "\x00"
  "ProductURL" "\x00"
  "SWBuildID" "\x00"
  "MainsVoltage" "\x00"
  "MainsFrequency" "\x00"
  "BatteryVoltage" "\x00"
  "BatteryPercentage" "\x00"
  "CurrentTemperature" "\x00"
  "MinTempExperienced" "\x00"
  "MaxTempExperienced" "\x00"
  "OverTempTotalDwell" "\x00"
  "IdentifyTime" "\x00"
  "GroupNameSupport" "\x00"
  "SceneCount" "\x00"
  "CurrentScene" "\x00"
  "CurrentGroup" "\x00"
  "SceneValid" "\x00"
  "Power" "\x00"
  "StartUpOnOff" "\x00"
  "SwitchType" "\x00"
  "Dimmer" "\x00"
  "DimmerOptions" "\x00"
  "DimmerRemainingTime" "\x00"
  "OnOffTransitionTime" "\x00"
  "AlarmCount" "\x00"
  "Time" "\x00"
  "TimeStatus" "\x00"
  "TimeZone" "\x00"
  "DstStart" "\x00"
  "DstEnd" "\x00"
  "DstShift" "\x00"
  "StandardTime" "\x00"
  "LocalTime" "\x00"
  "LastSetTime" "\x00"
  "ValidUntilTime" "\x00"
  "TimeEpoch" "\x00"
  "LocationType" "\x00"
  "LocationMethod" "\x00"
  "LocationAge" "\x00"
  "QualityMeasure" "\x00"
  "NumberOfDevices" "\x00"
  "AnalogInDescription" "\x00"
  "AnalogInMaxValue" "\x00"
  "AnalogInMinValue" "\x00"
  "AnalogInOutOfService" "\x00"
  "AqaraRotate" "\x00"
  "AnalogInReliability" "\x00"
  "AnalogInResolution" "\x00"
  "AnalogInStatusFlags" "\x00"
  "AnalogInEngineeringUnits" "\x00"
  "AnalogInApplicationType" "\x00"
  "Aqara_FF05" "\x00"
  "AnalogOutDescription" "\x00"
  "AnalogOutMaxValue" "\x00"
  "AnalogOutMinValue" "\x00"
  "AnalogOutOutOfService" "\x00"
  "AnalogOutValue" "\x00"
  "AnalogOutReliability" "\x00"
  "AnalogOutRelinquishDefault" "\x00"
  "AnalogOutResolution" "\x00"
  "AnalogOutStatusFlags" "\x00"
  "AnalogOutEngineeringUnits" "\x00"
  "AnalogOutApplicationType" "\x00"
  "AnalogDescription" "\x00"
  "AnalogOutOfService" "\x00"
  "AnalogValue" "\x00"
  "AnalogPriorityArray" "\x00"
  "AnalogReliability" "\x00"
  "AnalogRelinquishDefault" "\x00"
  "AnalogStatusFlags" "\x00"
  "AnalogEngineeringUnits" "\x00"
  "AnalogApplicationType" "\x00"
  "BinaryInActiveText" "\x00"
  "BinaryInDescription" "\x00"
  "BinaryInInactiveText" "\x00"
  "BinaryInOutOfService" "\x00"
  "BinaryInPolarity" "\x00"
  "BinaryInValue" "\x00"
  "BinaryInReliability" "\x00"
  "BinaryInStatusFlags" "\x00"
  "BinaryInApplicationType" "\x00"
  "BinaryOutActiveText" "\x00"
  "BinaryOutDescription" "\x00"
  "BinaryOutInactiveText" "\x00"
  "BinaryOutMinimumOffTime" "\x00"
  "BinaryOutMinimumOnTime" "\x00"
  "BinaryOutOutOfService" "\x00"
  "BinaryOutPolarity" "\x00"
  "BinaryOutValue" "\x00"
  "BinaryOutReliability" "\x00"
  "BinaryOutRelinquishDefault" "\x00"
  "BinaryOutStatusFlags" "\x00"
  "BinaryOutApplicationType" "\x00"
  "BinaryActiveText" "\x00"
  "BinaryDescription" "\x00"
  "BinaryInactiveText" "\x00"
  "BinaryMinimumOffTime" "\x00"
  "BinaryMinimumOnTime" "\x00"
  "BinaryOutOfService" "\x00"
  "BinaryValue" "\x00"
  "BinaryReliability" "\x00"
  "BinaryRelinquishDefault" "\x00"
  "BinaryStatusFlags" "\x00"
  "BinaryApplicationType" "\x00"
  "MultiInDescription" "\x00"
  "MultiInNumberOfStates" "\x00"
  "MultiInOutOfService" "\x00"
  "MultiInValue" "\x00"
  "MultiInReliability" "\x00"
  "MultiInStatusFlags" "\x00"
  "MultiInApplicationType" "\x00"
  "MultiOutDescription" "\x00"
  "MultiOutNumberOfStates" "\x00"
  "MultiOutOutOfService" "\x00"
  "MultiOutValue" "\x00"
  "MultiOutReliability" "\x00"
  "MultiOutRelinquishDefault" "\x00"
  "MultiOutStatusFlags" "\x00"
  "MultiOutApplicationType" "\x00"
  "MultiDescription" "\x00"
  "MultiNumberOfStates" "\x00"
  "MultiOutOfService" "\x00"
  "MultiValue" "\x00"
  "MultiReliability" "\x00"
  "MultiRelinquishDefault" "\x00"
  "MultiStatusFlags" "\x00"
  "MultiApplicationType" "\x00"
  "TotalProfileNum" "\x00"
  "MultipleScheduling" "\x00"
  "EnergyFormatting" "\x00"
  "EnergyRemote" "\x00"
  "ScheduleMode" "\x00"
  "CheckinInterval" "\x00"
  "LongPollInterval" "\x00"
  "ShortPollInterval" "\x00"
  "FastPollTimeout" "\x00"
  "CheckinIntervalMin" "\x00"
  "LongPollIntervalMin" "\x00"
  "FastPollTimeoutMax" "\x00"
  "PhysicalClosedLimit" "\x00"
  "MotorStepSize" "\x00"
  "Status" "\x00"
  "ClosedLimit" "\x00"
  "Mode" "\x00"
  "LockState" "\x00"
  "LockType" "\x00"
  "ActuatorEnabled" "\x00"
  "DoorState" "\x00"
  "DoorOpenEvents" "\x00"
  "DoorClosedEvents" "\x00"
  "OpenPeriod" "\x00"
  "AqaraVibrationMode" "\x00"
  "AqaraVibrationsOrAngle" "\x00"
  "AqaraVibration505" "\x00"
  "AqaraAccelerometer" "\x00"
  "WindowCoveringType" "\x00"
  "PhysicalClosedLimitLift" "\x00"
  "PhysicalClosedLimitTilt" "\x00"
  "CurrentPositionLift" "\x00"
  "CurrentPositionTilt" "\x00"
  "NumberofActuationsLift" "\x00"
  "NumberofActuationsTilt" "\x00"
  "ConfigStatus" "\x00"
  "CurrentPositionLiftPercentage" "\x00"
  "CurrentPositionTiltPercentage" "\x00"
  "InstalledOpenLimitLift" "\x00"
  "InstalledClosedLimitLift" "\x00"
  "InstalledOpenLimitTilt" "\x00"
  "InstalledClosedLimitTilt" "\x00"
  "VelocityLift" "\x00"
  "AccelerationTimeLift" "\x00"
  "DecelerationTimeLift" "\x00"
  "IntermediateSetpointsLift" "\x00"
  "IntermediateSetpointsTilt" "\x00"
  "Hue" "\x00"
  "Sat" "\x00"
  "RemainingTime" "\x00"
  "X" "\x00"
  "Y" "\x00"
  "DriftCompensation" "\x00"
  "CompensationText" "\x00"
  "CT" "\x00"
  "ColorMode" "\x00"
  "NumberOfPrimaries" "\x00"
  "Primary1X" "\x00"
  "Primary1Y" "\x00"
  "Primary1Intensity" "\x00"
  "Primary2X" "\x00"
  "Primary2Y" "\x00"
  "Primary2Intensity" "\x00"
  "Primary3X" "\x00"
  "Primary3Y" "\x00"
  "Primary3Intensity" "\x00"
  "WhitePointX" "\x00"
  "WhitePointY" "\x00"
  "ColorPointRX" "\x00"
  "ColorPointRY" "\x00"
  "ColorPointRIntensity" "\x00"
  "ColorPointGX" "\x00"
  "ColorPointGY" "\x00"
  "ColorPointGIntensity" "\x00"
  "ColorPointBX" "\x00"
  "ColorPointBY" "\x00"
  "ColorPointBIntensity" "\x00"
  "Illuminance" "\x00"
  "IlluminanceMinMeasuredValue" "\x00"
  "IlluminanceMaxMeasuredValue" "\x00"
  "IlluminanceTolerance" "\x00"
  "IlluminanceLightSensorType" "\x00"
  "IlluminanceLevelStatus" "\x00"
  "IlluminanceTargetLevel" "\x00"
  "Temperature" "\x00"
  "TemperatureMinMeasuredValue" "\x00"
  "TemperatureMaxMeasuredValue" "\x00"
  "TemperatureTolerance" "\x00"
  "PressureUnit" "\x00"
  "Pressure" "\x00"
  "PressureMinMeasuredValue" "\x00"
  "PressureMaxMeasuredValue" "\x00"
  "PressureTolerance" "\x00"
  "PressureScaledValue" "\x00"
  "PressureMinScaledValue" "\x00"
  "PressureMaxScaledValue" "\x00"
  "PressureScaledTolerance" "\x00"
  "PressureScale" "\x00"
  "FlowRate" "\x00"
  "FlowMinMeasuredValue" "\x00"
  "FlowMaxMeasuredValue" "\x00"
  "FlowTolerance" "\x00"
  "Humidity" "\x00"
  "HumidityMinMeasuredValue" "\x00"
  "HumidityMaxMeasuredValue" "\x00"
  "HumidityTolerance" "\x00"
  "Occupancy" "\x00"
  "OccupancySensorType" "\x00"
  "ZoneState" "\x00"
  "ZoneType" "\x00"
  "ZoneStatus" "\x00"
  "CurrentSummDelivered" "\x00"
  "CompanyName" "\x00"
  "MeterTypeID" "\x00"
  "DataQualityID" "\x00"
  "CustomerName" "\x00"
  "Model" "\x00"
  "PartNumber" "\x00"
  "ProductRevision" "\x00"
  "SoftwareRevision" "\x00"
  "UtilityName" "\x00"
  "POD" "\x00"
  "AvailablePower" "\x00"
  "PowerThreshold" "\x00"
  "RMSVoltage" "\x00"
  "RMSCurrent" "\x00"
  "ActivePower" "\x00"
  "NumberOfResets" "\x00"
  "PersistentMemoryWrites" "\x00"
  "LastMessageLQI" "\x00"
  "LastMessageRSSI" "\x00"
  "Identify" "\x00"
  "xxxx" "\x00"
  "IdentifyQuery" "\x00"
  "AddGroup" "\x00"
  "xxxx00" "\x00"
  "ViewGroup" "\x00"
  "GetGroup" "\x00"
  "01xxxx" "\x00"
  "GetAllGroups" "\x00"
  "00" "\x00"
  "RemoveGroup" "\x00"
  "RemoveAllGroups" "\x00"
  "ViewScene" "\x00"
  "xxxxyy" "\x00"
  "RemoveScene" "\x00"
  "RemoveAllScenes" "\x00"
  "RecallScene" "\x00"
  "GetSceneMembership" "\x00"
  "PowerOffEffect" "\x00"
  "xxyy" "\x00"
  "PowerOnRecall" "\x00"
  "PowerOnTimer" "\x00"
  "xxyyyyzzzz" "\x00"
  "xx0A00" "\x00"
  "DimmerUp" "\x00"
  "00190200" "\x00"
  "DimmerDown" "\x00"
  "01190200" "\x00"
  "DimmerStop" "\x00"
  "ResetAlarm" "\x00"
  "xxyyyy" "\x00"
  "ResetAllAlarms" "\x00"
  "xx000A00" "\x00"
  "HueSat" "\x00"
  "xxyy0A00" "\x00"
  "Color" "\x00"
  "xxxxyyyy0A00" "\x00"
  "xxxx0A00" "\x00"
  "ShutterOpen" "\x00"
  "ShutterClose" "\x00"
  "ShutterStop" "\x00"
  "ShutterLift" "\x00"
  "xx" "\x00"
  "ShutterTilt" "\x00"
  "Shutter" "\x00"
  "DimmerMove" "\x00"
  "xx0A" "\x00"
  "DimmerStepUp" "\x00"
  "00xx0A00" "\x00"
  "DimmerStepDown" "\x00"
  "01xx0A00" "\x00"
  "DimmerStep" "\x00"
  "xx190A00" "\x00"
  "01" "\x00"
  "HueMove" "\x00"
  "xx19" "\x00"
  "HueStepUp" "\x00"
  "HueStepDown" "\x00"
  "03xx0A00" "\x00"
  "HueStep" "\x00"
  "SatMove" "\x00"
  "SatStep" "\x00"
  "xx190A" "\x00"
  "ColorMove" "\x00"
  "xxxxyyyy" "\x00"
  "ColorStep" "\x00"
  "ColorTempMoveUp" "\x00"
  "01xxxx000000000000" "\x00"
  "ColorTempMoveDown" "\x00"
  "03xxxx000000000000" "\x00"
  "ColorTempMoveStop" "\x00"
  "00xxxx000000000000" "\x00"
  "ColorTempMove" "\x00"
  "xxyyyy000000000000" "\x00"
  "ColorTempStepUp" "\x00"
  "01xxxx0A0000000000" "\x00"
  "ColorTempStepDown" "\x00"
  "03xxxx0A0000000000" "\x00"
  "ColorTempStep" "\x00"
  "xxyyyy0A0000000000" "\x00"
  "ArrowClick" "\x00"
  "ArrowHold" "\x00"
  "ArrowRelease" "\x00"
  "ZoneStatusChange" "\x00"
  "xxxxyyzz" "\x00"
  "xxyyzzzz" "\x00"
  "AddScene" "\x00"
  "xxyyyyzz" "\x00"
  "StoreScene" "\x00"
  ;

enum Z_offsets {
  Zo_ = 0,
  Zo_ZCLVersion = 1,
  Zo_AppVersion = 12,
  Zo_StackVersion = 23,
  Zo_HWVersion = 36,
  Zo_Manufacturer = 46,
  Zo_ModelId = 59,
  Zo_DateCode = 67,
  Zo_PowerSource = 76,
  Zo_GenericDeviceClass = 88,
  Zo_GenericDeviceType = 107,
  Zo_ProductCode = 125,
  Zo_ProductURL = 137,
  Zo_SWBuildID = 148,
  Zo_MainsVoltage = 158,
  Zo_MainsFrequency = 171,
  Zo_BatteryVoltage = 186,
  Zo_BatteryPercentage = 201,
  Zo_CurrentTemperature = 219,
  Zo_MinTempExperienced = 238,
  Zo_MaxTempExperienced = 257,
  Zo_OverTempTotalDwell = 276,
  Zo_IdentifyTime = 295,
  Zo_GroupNameSupport = 308,
  Zo_SceneCount = 325,
  Zo_CurrentScene = 336,
  Zo_CurrentGroup = 349,
  Zo_SceneValid = 362,
  Zo_Power = 373,
  Zo_StartUpOnOff = 379,
  Zo_SwitchType = 392,
  Zo_Dimmer = 403,
  Zo_DimmerOptions = 410,
  Zo_DimmerRemainingTime = 424,
  Zo_OnOffTransitionTime = 444,
  Zo_AlarmCount = 464,
  Zo_Time = 475,
  Zo_TimeStatus = 480,
  Zo_TimeZone = 491,
  Zo_DstStart = 500,
  Zo_DstEnd = 509,
  Zo_DstShift = 516,
  Zo_StandardTime = 525,
  Zo_LocalTime = 538,
  Zo_LastSetTime = 548,
  Zo_ValidUntilTime = 560,
  Zo_TimeEpoch = 575,
  Zo_LocationType = 585,
  Zo_LocationMethod = 598,
  Zo_LocationAge = 613,
  Zo_QualityMeasure = 625,
  Zo_NumberOfDevices = 640,
  Zo_AnalogInDescription = 656,
  Zo_AnalogInMaxValue = 676,
  Zo_AnalogInMinValue = 693,
  Zo_AnalogInOutOfService = 710,
  Zo_AqaraRotate = 731,
  Zo_AnalogInReliability = 743,
  Zo_AnalogInResolution = 763,
  Zo_AnalogInStatusFlags = 782,
  Zo_AnalogInEngineeringUnits = 802,
  Zo_AnalogInApplicationType = 827,
  Zo_Aqara_FF05 = 851,
  Zo_AnalogOutDescription = 862,
  Zo_AnalogOutMaxValue = 883,
  Zo_AnalogOutMinValue = 901,
  Zo_AnalogOutOutOfService = 919,
  Zo_AnalogOutValue = 941,
  Zo_AnalogOutReliability = 956,
  Zo_AnalogOutRelinquishDefault = 977,
  Zo_AnalogOutResolution = 1004,
  Zo_AnalogOutStatusFlags = 1024,
  Zo_AnalogOutEngineeringUnits = 1045,
  Zo_AnalogOutApplicationType = 1071,
  Zo_AnalogDescription = 1096,
  Zo_AnalogOutOfService = 1114,
  Zo_AnalogValue = 1133,
  Zo_AnalogPriorityArray = 1145,
  Zo_AnalogReliability = 1165,
  Zo_AnalogRelinquishDefault = 1183,
  Zo_AnalogStatusFlags = 1207,
  Zo_AnalogEngineeringUnits = 1225,
  Zo_AnalogApplicationType = 1248,
  Zo_BinaryInActiveText = 1270,
  Zo_BinaryInDescription = 1289,
  Zo_BinaryInInactiveText = 1309,
  Zo_BinaryInOutOfService = 1330,
  Zo_BinaryInPolarity = 1351,
  Zo_BinaryInValue = 1368,
  Zo_BinaryInReliability = 1382,
  Zo_BinaryInStatusFlags = 1402,
  Zo_BinaryInApplicationType = 1422,
  Zo_BinaryOutActiveText = 1446,
  Zo_BinaryOutDescription = 1466,
  Zo_BinaryOutInactiveText = 1487,
  Zo_BinaryOutMinimumOffTime = 1509,
  Zo_BinaryOutMinimumOnTime = 1533,
  Zo_BinaryOutOutOfService = 1556,
  Zo_BinaryOutPolarity = 1578,
  Zo_BinaryOutValue = 1596,
  Zo_BinaryOutReliability = 1611,
  Zo_BinaryOutRelinquishDefault = 1632,
  Zo_BinaryOutStatusFlags = 1659,
  Zo_BinaryOutApplicationType = 1680,
  Zo_BinaryActiveText = 1705,
  Zo_BinaryDescription = 1722,
  Zo_BinaryInactiveText = 1740,
  Zo_BinaryMinimumOffTime = 1759,
  Zo_BinaryMinimumOnTime = 1780,
  Zo_BinaryOutOfService = 1800,
  Zo_BinaryValue = 1819,
  Zo_BinaryReliability = 1831,
  Zo_BinaryRelinquishDefault = 1849,
  Zo_BinaryStatusFlags = 1873,
  Zo_BinaryApplicationType = 1891,
  Zo_MultiInDescription = 1913,
  Zo_MultiInNumberOfStates = 1932,
  Zo_MultiInOutOfService = 1954,
  Zo_MultiInValue = 1974,
  Zo_MultiInReliability = 1987,
  Zo_MultiInStatusFlags = 2006,
  Zo_MultiInApplicationType = 2025,
  Zo_MultiOutDescription = 2048,
  Zo_MultiOutNumberOfStates = 2068,
  Zo_MultiOutOutOfService = 2091,
  Zo_MultiOutValue = 2112,
  Zo_MultiOutReliability = 2126,
  Zo_MultiOutRelinquishDefault = 2146,
  Zo_MultiOutStatusFlags = 2172,
  Zo_MultiOutApplicationType = 2192,
  Zo_MultiDescription = 2216,
  Zo_MultiNumberOfStates = 2233,
  Zo_MultiOutOfService = 2253,
  Zo_MultiValue = 2271,
  Zo_MultiReliability = 2282,
  Zo_MultiRelinquishDefault = 2299,
  Zo_MultiStatusFlags = 2322,
  Zo_MultiApplicationType = 2339,
  Zo_TotalProfileNum = 2360,
  Zo_MultipleScheduling = 2376,
  Zo_EnergyFormatting = 2395,
  Zo_EnergyRemote = 2412,
  Zo_ScheduleMode = 2425,
  Zo_CheckinInterval = 2438,
  Zo_LongPollInterval = 2454,
  Zo_ShortPollInterval = 2471,
  Zo_FastPollTimeout = 2489,
  Zo_CheckinIntervalMin = 2505,
  Zo_LongPollIntervalMin = 2524,
  Zo_FastPollTimeoutMax = 2544,
  Zo_PhysicalClosedLimit = 2563,
  Zo_MotorStepSize = 2583,
  Zo_Status = 2597,
  Zo_ClosedLimit = 2604,
  Zo_Mode = 2616,
  Zo_LockState = 2621,
  Zo_LockType = 2631,
  Zo_ActuatorEnabled = 2640,
  Zo_DoorState = 2656,
  Zo_DoorOpenEvents = 2666,
  Zo_DoorClosedEvents = 2681,
  Zo_OpenPeriod = 2698,
  Zo_AqaraVibrationMode = 2709,
  Zo_AqaraVibrationsOrAngle = 2728,
  Zo_AqaraVibration505 = 2751,
  Zo_AqaraAccelerometer = 2769,
  Zo_WindowCoveringType = 2788,
  Zo_PhysicalClosedLimitLift = 2807,
  Zo_PhysicalClosedLimitTilt = 2831,
  Zo_CurrentPositionLift = 2855,
  Zo_CurrentPositionTilt = 2875,
  Zo_NumberofActuationsLift = 2895,
  Zo_NumberofActuationsTilt = 2918,
  Zo_ConfigStatus = 2941,
  Zo_CurrentPositionLiftPercentage = 2954,
  Zo_CurrentPositionTiltPercentage = 2984,
  Zo_InstalledOpenLimitLift = 3014,
  Zo_InstalledClosedLimitLift = 3037,
  Zo_InstalledOpenLimitTilt = 3062,
  Zo_InstalledClosedLimitTilt = 3085,
  Zo_VelocityLift = 3110,
  Zo_AccelerationTimeLift = 3123,
  Zo_DecelerationTimeLift = 3144,
  Zo_IntermediateSetpointsLift = 3165,
  Zo_IntermediateSetpointsTilt = 3191,
  Zo_Hue = 3217,
  Zo_Sat = 3221,
  Zo_RemainingTime = 3225,
  Zo_X = 3239,
  Zo_Y = 3241,
  Zo_DriftCompensation = 3243,
  Zo_CompensationText = 3261,
  Zo_CT = 3278,
  Zo_ColorMode = 3281,
  Zo_NumberOfPrimaries = 3291,
  Zo_Primary1X = 3309,
  Zo_Primary1Y = 3319,
  Zo_Primary1Intensity = 3329,
  Zo_Primary2X = 3347,
  Zo_Primary2Y = 3357,
  Zo_Primary2Intensity = 3367,
  Zo_Primary3X = 3385,
  Zo_Primary3Y = 3395,
  Zo_Primary3Intensity = 3405,
  Zo_WhitePointX = 3423,
  Zo_WhitePointY = 3435,
  Zo_ColorPointRX = 3447,
  Zo_ColorPointRY = 3460,
  Zo_ColorPointRIntensity = 3473,
  Zo_ColorPointGX = 3494,
  Zo_ColorPointGY = 3507,
  Zo_ColorPointGIntensity = 3520,
  Zo_ColorPointBX = 3541,
  Zo_ColorPointBY = 3554,
  Zo_ColorPointBIntensity = 3567,
  Zo_Illuminance = 3588,
  Zo_IlluminanceMinMeasuredValue = 3600,
  Zo_IlluminanceMaxMeasuredValue = 3628,
  Zo_IlluminanceTolerance = 3656,
  Zo_IlluminanceLightSensorType = 3677,
  Zo_IlluminanceLevelStatus = 3704,
  Zo_IlluminanceTargetLevel = 3727,
  Zo_Temperature = 3750,
  Zo_TemperatureMinMeasuredValue = 3762,
  Zo_TemperatureMaxMeasuredValue = 3790,
  Zo_TemperatureTolerance = 3818,
  Zo_PressureUnit = 3839,
  Zo_Pressure = 3852,
  Zo_PressureMinMeasuredValue = 3861,
  Zo_PressureMaxMeasuredValue = 3886,
  Zo_PressureTolerance = 3911,
  Zo_PressureScaledValue = 3929,
  Zo_PressureMinScaledValue = 3949,
  Zo_PressureMaxScaledValue = 3972,
  Zo_PressureScaledTolerance = 3995,
  Zo_PressureScale = 4019,
  Zo_FlowRate = 4033,
  Zo_FlowMinMeasuredValue = 4042,
  Zo_FlowMaxMeasuredValue = 4063,
  Zo_FlowTolerance = 4084,
  Zo_Humidity = 4098,
  Zo_HumidityMinMeasuredValue = 4107,
  Zo_HumidityMaxMeasuredValue = 4132,
  Zo_HumidityTolerance = 4157,
  Zo_Occupancy = 4175,
  Zo_OccupancySensorType = 4185,
  Zo_ZoneState = 4205,
  Zo_ZoneType = 4215,
  Zo_ZoneStatus = 4224,
  Zo_CurrentSummDelivered = 4235,
  Zo_CompanyName = 4256,
  Zo_MeterTypeID = 4268,
  Zo_DataQualityID = 4280,
  Zo_CustomerName = 4294,
  Zo_Model = 4307,
  Zo_PartNumber = 4313,
  Zo_ProductRevision = 4324,
  Zo_SoftwareRevision = 4340,
  Zo_UtilityName = 4357,
  Zo_POD = 4369,
  Zo_AvailablePower = 4373,
  Zo_PowerThreshold = 4388,
  Zo_RMSVoltage = 4403,
  Zo_RMSCurrent = 4414,
  Zo_ActivePower = 4425,
  Zo_NumberOfResets = 4437,
  Zo_PersistentMemoryWrites = 4452,
  Zo_LastMessageLQI = 4475,
  Zo_LastMessageRSSI = 4490,
  Zo_Identify = 4506,
  Zo_xxxx = 4515,
  Zo_IdentifyQuery = 4520,
  Zo_AddGroup = 4534,
  Zo_xxxx00 = 4543,
  Zo_ViewGroup = 4550,
  Zo_GetGroup = 4560,
  Zo_01xxxx = 4569,
  Zo_GetAllGroups = 4576,
  Zo_00 = 4589,
  Zo_RemoveGroup = 4592,
  Zo_RemoveAllGroups = 4604,
  Zo_ViewScene = 4620,
  Zo_xxxxyy = 4630,
  Zo_RemoveScene = 4637,
  Zo_RemoveAllScenes = 4649,
  Zo_RecallScene = 4665,
  Zo_GetSceneMembership = 4677,
  Zo_PowerOffEffect = 4696,
  Zo_xxyy = 4711,
  Zo_PowerOnRecall = 4716,
  Zo_PowerOnTimer = 4730,
  Zo_xxyyyyzzzz = 4743,
  Zo_xx0A00 = 4754,
  Zo_DimmerUp = 4761,
  Zo_00190200 = 4770,
  Zo_DimmerDown = 4779,
  Zo_01190200 = 4790,
  Zo_DimmerStop = 4799,
  Zo_ResetAlarm = 4810,
  Zo_xxyyyy = 4821,
  Zo_ResetAllAlarms = 4828,
  Zo_xx000A00 = 4843,
  Zo_HueSat = 4852,
  Zo_xxyy0A00 = 4859,
  Zo_Color = 4868,
  Zo_xxxxyyyy0A00 = 4874,
  Zo_xxxx0A00 = 4887,
  Zo_ShutterOpen = 4896,
  Zo_ShutterClose = 4908,
  Zo_ShutterStop = 4921,
  Zo_ShutterLift = 4933,
  Zo_xx = 4945,
  Zo_ShutterTilt = 4948,
  Zo_Shutter = 4960,
  Zo_DimmerMove = 4968,
  Zo_xx0A = 4979,
  Zo_DimmerStepUp = 4984,
  Zo_00xx0A00 = 4997,
  Zo_DimmerStepDown = 5006,
  Zo_01xx0A00 = 5021,
  Zo_DimmerStep = 5030,
  Zo_xx190A00 = 5041,
  Zo_01 = 5050,
  Zo_HueMove = 5053,
  Zo_xx19 = 5061,
  Zo_HueStepUp = 5066,
  Zo_HueStepDown = 5076,
  Zo_03xx0A00 = 5088,
  Zo_HueStep = 5097,
  Zo_SatMove = 5105,
  Zo_SatStep = 5113,
  Zo_xx190A = 5121,
  Zo_ColorMove = 5128,
  Zo_xxxxyyyy = 5138,
  Zo_ColorStep = 5147,
  Zo_ColorTempMoveUp = 5157,
  Zo_01xxxx000000000000 = 5173,
  Zo_ColorTempMoveDown = 5192,
  Zo_03xxxx000000000000 = 5210,
  Zo_ColorTempMoveStop = 5229,
  Zo_00xxxx000000000000 = 5247,
  Zo_ColorTempMove = 5266,
  Zo_xxyyyy000000000000 = 5280,
  Zo_ColorTempStepUp = 5299,
  Zo_01xxxx0A0000000000 = 5315,
  Zo_ColorTempStepDown = 5334,
  Zo_03xxxx0A0000000000 = 5352,
  Zo_ColorTempStep = 5371,
  Zo_xxyyyy0A0000000000 = 5385,
  Zo_ArrowClick = 5404,
  Zo_ArrowHold = 5415,
  Zo_ArrowRelease = 5425,
  Zo_ZoneStatusChange = 5438,
  Zo_xxxxyyzz = 5455,
  Zo_xxyyzzzz = 5464,
  Zo_AddScene = 5473,
  Zo_xxyyyyzz = 5482,
  Zo_StoreScene = 5491,
};


/*
  DO NOT EDIT
*/


#endif // USE_ZIGBEE
