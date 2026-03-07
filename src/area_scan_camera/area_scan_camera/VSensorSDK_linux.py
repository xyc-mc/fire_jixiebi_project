import platform
import ctypes
from ctypes import *
from threading import local
import os

# 检查操作系统，如果不是Linux则抛出错误
if platform.system() != "Linux":
    raise SystemError("This SDK only supports Linux operating system")

#回调函数类型
CALLBACK_FUNC_TYPE = CFUNCTYPE
#SDK动态库
_sdk = None

def _Init():
    global _sdk
    global CALLBACK_FUNC_TYPE
    
    _sdk = cdll.LoadLibrary("libVSensorSDK.so")
    CALLBACK_FUNC_TYPE = CFUNCTYPE

_Init()

#错误码
CAMERA_STATUS_SUCCESS                          =  0    #操作成功
CAMERA_STATUS_FAILED                           = -1    #操作失败
CAMERA_STATUS_INTERNAL_ERROR                   = -2    #内部错误
CAMERA_STATUS_UNKNOW                           = -3    #未知错误
CAMERA_STATUS_NOT_SUPPORTED                    = -4    #不支持该功能
CAMERA_STATUS_NOT_INITIALIZED                  = -5    #初始化未完成
CAMERA_STATUS_PARAMETER_INVALID                = -6    #参数无效
CAMERA_STATUS_PARAMETER_OUT_OF_BOUND           = -7    #参数越界
CAMERA_STATUS_UNENABLED                        = -8    #未使能
CAMERA_STATUS_USER_CANCEL                      = -9    #用户手动取消了，比如roi面板点击取消，返回
CAMERA_STATUS_PATH_NOT_FOUND                   = -10   #注册表中没有找到对应的路径
CAMERA_STATUS_SIZE_DISMATCH                    = -11   #获得图像数据长度和定义的尺寸不匹配 
CAMERA_STATUS_TIME_OUT                         = -12   #超时错误
CAMERA_STATUS_IO_ERROR                         = -13   #硬件IO错误 
CAMERA_STATUS_COMM_ERROR                       = -14   #通讯错误 
CAMERA_STATUS_BUS_ERROR                        = -15   #总线错误
CAMERA_STATUS_NO_DEVICE_FOUND                  = -16   #没有发现设备
CAMERA_STATUS_NO_LOGIC_DEVICE_FOUND            = -17   #未找到逻辑设备
CAMERA_STATUS_DEVICE_IS_OPENED                 = -18   #设备已经打开 
CAMERA_STATUS_DEVICE_IS_CLOSED                 = -19   #设备已经关闭
CAMERA_STATUS_DEVICE_VEDIO_CLOSED              = -20   #没有打开设备视频，调用录像相关的函数时，如果相机视频没有打开，则回返回该错误。 
CAMERA_STATUS_FILE_CREATE_FAILED               = -22   #创建文件失败 
CAMERA_STATUS_FILE_INVALID                     = -23   #文件格式无效
CAMERA_STATUS_WRITE_PROTECTED                  = -24   #写保护，不可写
CAMERA_STATUS_GRAB_FAILED                      = -25   #数据采集失败 
CAMERA_STATUS_LOST_DATA                        = -26   #数据丢失，不完整
CAMERA_STATUS_EOF_ERROR                        = -27   #未接收到帧结束符 
CAMERA_STATUS_BUSY                             = -28   #正忙(上一次操作还在进行中)，此次操作不能进行 
CAMERA_STATUS_WAIT                             = -29   #需要等待(进行操作的条件不成立)，可以再次尝试 
CAMERA_STATUS_IN_PROCESS                       = -30   #正在进行，已经被操作过 
CAMERA_STATUS_IIC_ERROR                        = -31   #IIC传输错误 
CAMERA_STATUS_SPI_ERROR                        = -32   #SPI传输错误 
CAMERA_STATUS_USB_CONTROL_ERROR                = -33   #USB控制传输错误 
CAMERA_STATUS_USB_BULK_ERROR                   = -34   #USB BULK传输错误
CAMERA_STATUS_SOCKET_INIT_ERROR                = -35   #网络传输套件初始化失败
CAMERA_STATUS_GIGE_FILTER_INIT_ERROR           = -36   #网络相机内核过滤驱动初始化失败，请检查是否正确安装了驱动，或者重新安装。 
CAMERA_STATUS_NET_SEND_ERROR                   = -37   #网络数据发送错误 
CAMERA_STATUS_DEVICE_LOST                      = -38   #与网络相机失去连接，心跳检测超时 
CAMERA_STATUS_DATA_RECV_LESS                   = -39   #接收到的字节数比请求的少
CAMERA_STATUS_FUNCTION_LOAD_FAILED             = -40   #从文件中加载程序失败 
CAMERA_STATUS_CRITICAL_FILE_LOST               = -41   #程序运行所必须的文件丢失。
CAMERA_STATUS_SENSOR_ID_DISMATCH               = -42   #固件和程序不匹配，原因是下载了错误的固件。 
CAMERA_STATUS_OUT_OF_RANGE                     = -43   #参数超出有效范围。
CAMERA_STATUS_REGISTRY_ERROR                   = -44   #安装程序注册错误。请重新安装程序，或者运行安装目录Setup/Installer.exe 
CAMERA_STATUS_ACCESS_DENY                      = -45   #禁止访问。指定相机已经被其他程序占用时，再申请访问该相机，会返回该状态。(一个相机不能被多个程序同时访问)
CAMERA_STATUS_CAMERA_NEED_RESET                = -46   #表示相机需要复位后才能正常使用，此时请让相机断电重启，或者重启操作系统后，便可正常使用。
CAMERA_STATUS_ISP_MOUDLE_NOT_INITIALIZED       = -47   #ISP模块未初始化
CAMERA_STATUS_ISP_DATA_CRC_ERROR               = -48   #数据校验错误 
CAMERA_STATUS_MV_TEST_FAILED                   = -49   #数据测试失败
CAMERA_STATUS_INTERNAL_ERR1                    = -50   #内部错误1 
CAMERA_STATUS_U3V_NO_CONTROL_EP                = -51   #U3V控制端点未找到 
CAMERA_STATUS_U3V_CONTROL_ERROR                = -52   #U3V控制通讯错误
CAMERA_STATUS_INVALID_FRIENDLY_NAME            = -53   #无效的设备名，名字里不能包含以下字符(\/:*?"<>|") 
CAMERA_STATUS_FORMAT_ERROR                     = -54   #格式错误
CAMERA_STATUS_PCIE_OPEN_ERROR                  = -55   #PCIE设备打开失败
CAMERA_STATUS_PCIE_COMM_ERROR                  = -56   #PCIE设备通讯失败
CAMERA_STATUS_PCIE_DDR_ERROR                   = -57   #PCIE DDR错误
CAMERA_STATUS_IP_ERROR                         = -58   #IP错误
CAMERA_AIA_PACKET_RESEND                       = 256   #该帧需要重传
CAMERA_AIA_NOT_IMPLEMENTED                     = 32769 #设备不支持的命令
CAMERA_AIA_INVALID_PARAMETER                   = 32770 #命令参数非法
CAMERA_AIA_INVALID_ADDRESS                     = 32771 #不可访问的地址
CAMERA_AIA_WRITE_PROTECT                       = 32772 #访问的对象不可写
CAMERA_AIA_BAD_ALIGNMENT                       = 32773 #访问的地址没有按照要求对齐
CAMERA_AIA_ACCESS_DENIED                       = 32774 #没有访问权限
CAMERA_AIA_BUSY                                = 32775 #命令正在处理中
CAMERA_AIA_DEPRECATED                          = 32776 #0x8008-0x0800B  0x800F该指令已经废弃
CAMERA_AIA_PACKET_UNAVAILABLE                  = 32780 #包无效
CAMERA_AIA_DATA_OVERRUN                        = 32781 #数据溢出，通常是收到的数据比需要的多
CAMERA_AIA_INVALID_HEADER                      = 32782 #数据包头部中某些区域与协议不匹配
CAMERA_AIA_PACKET_NOT_YET_AVAILABLE            = 32784 #图像分包数据还未准备好，多用于触发模式，应用程序访问超时
CAMERA_AIA_PACKET_AND_PREV_REMOVED_FROM_MEMORY = 32785 #需要访问的分包已经不存在。多用于重传时数据已经不在缓冲区中
CAMERA_AIA_PACKET_REMOVED_FROM_MEMORY          = 32786 #CAMERA_AIA_PACKET_AND_PREV_REMOVED_FROM_MEMORY
CAMERA_AIA_NO_REF_TIME                         = 2067  #没有参考时钟源。多用于时间同步的命令执行时
CAMERA_AIA_PACKET_TEMPORARILY_UNAVAILABLE      = 2068  #由于信道带宽问题，当前分包暂时不可用，需稍后进行访问
CAMERA_AIA_OVERFLOW                            = 2069  #设备端数据溢出，通常是队列已满
CAMERA_AIA_ACTION_LATE                         = 2070  #命令执行已经超过有效的指定时间
CAMERA_AIA_ERROR                               = 36863 #错误
VSENSOR_STATUS_NO_DEVICE_FOUND                 = -100  #未连接设备 
VSENSOR_STATUS_SEND_INSTRUTION_FAILED          = -101  #串口发送指令失败
VSENSOR_STATUS_RECEIVE_INSTRUTION_FAILED       = -102  #串口接收指令失败
VSENSOR_STATUS_PARA_RECEIVE_FAILED             = -103  #读取标定文件失败
VSENSOR_STATUS_SERIAL_NUMBER_ERROR             = -104  #相机编号异常
VSENSOR_STATUS_JUMBOFRAME_CONNECT_FAILED       = -105  #未打开巨型帧
VSENSOR_STATUS_GIGE_CONNECT_FAILED             = -106  #非千兆网通信
VSENSOR_STATUS_USB3_CONNECT_FAILED             = -107  #未连接USB3.0接口
VSENSOR_STATUS_PREVIEW_IMAGE_FAILED            = -108  #预览图采集失败
VSENSOR_STATUS_CAPTURE_IMAGE_FAILED            = -109  #采集条纹图失败
VSENSOR_STATUS_CONTINUOUS_MODE_FAILED          = -110  #非连续采集模式
VSENSOR_STATUS_CONTINUOUSCAPTURE_TIMEOUT       = -111  #连续采集获取点云超时 
VSENSOR_STATUS_FILE_SAVE_FAILED                = -112  #文件保存出错 
VSENSOR_STATUS_FILE_FORMAT_FAILED              = -113  #保存文件格式有误
VSENSOR_STATUS_PARA_OUTOF_RANGE                = -114  #参数不在范围内 
VSENSOR_STATUS_CAPTURING_FAILED                = -115  #相机正在采集
VSENSOR_STATUS_CAPTUREMODE_FAILED              = -116  #相机采集模式有误 
VSENSOR_STATUS_FUNCTION_NOTSUPPERT             = -117  #该设备不支持该功能
VSENSOR_STATUS_DOWNSAMPLING_FAILED             = -118  #降采样打开失败，请先关闭ROI模式
VSENSOR_STATUS_ROI_FAILED                      = -119  #ROI打开失败，请先关闭降采样模式
VSENSOR_STATUS_NET_INIT_ERROR                  = -120  #网络通信创建失败
VSENSOR_STATUS_NET_SEND_ERROR                  = -121  #网络通信发送失败
VSENSOR_STATUS_NET_REV_ERROR                   = -122  #网络通信接收失败
VSENSOR_STATUS_SENSORTEMP_OVERTHRESHOLD        = -123  #传感器温度超过阈值
VSENSOR_STATUS_FILEPATH_READ_FAILED            = -124  #文件路径读取失败
VSENSOR_STATUS_FILECOMMENT_READ_FAILED         = -125  #文件内容读取失败
VSENSOR_STATUS_COM_INIT_ERROR                  = -126  #串口通信创建失败
VSENSOR_STATUS_COM_SEND_ERROR                  = -127  #串口通信发送失败
VSENSOR_STATUS_COM_REV_ERROR                   = -128  #串口通信接收失败
VSENSOR_STATUS_CAMERA_MEMORY_ERR               = -129  #相机存储空间错误
VSENSOR_STATUS_CAPTURE_DATA_OVERRUN            = -130  #数据溢出，通常是场景复杂导致
VSENSOR_STATUS_EXCEPTION_HANDING               = -140  #运行异常捕获
VSENSOR_STATUS_UNKNOWN_EXCEPTION               = -141  #运行未知错误
MODULE_STATUS_VOLTAGE_ERROR                    = -200  #供电电压异常
MODULE_STATUS_NO_PD_ERROR                      = -201  #无PD信号
MODULE_STATUS_PD_ERROR                         = -202  #PD信号异常
MODULE_STATUS_OVERHEATING_ERROR                = -203  #模组过热
MODULE_STATUS_REGISTER12_ERROR                 = -204  #12寄存器异常
MODULE_STATUS_REGISTER13_ERROR                 = -205  #13寄存器异常 
MODULE_STATUS_REGISTER33_ERROR                 = -206  #33寄存器异常
MODULE_STATUS_TRIGGER_ERROR                    = -207  #触发信号幅值异常
MODULE_STATUS_NO_TRIGGER_ERROR                 = -208  #无硬件触发信号
MODULE_STATUS_UART_ERROR                       = -209  #UART返回值异常 
MODULE_STATUS_LASER_DAMPING_ERROR              = -210  #激光器衰减异常
MODULE_STATUS_MEMS_ERROR                       = -211  #MEMS频率异常
MODULE_STATUS_I2C_ERROR                        = -212  #I2C通信错误
TYPE_GRAY                   = 0     #灰度相机
TYPE_RGB                    = 1     #彩色相机
TYPE_ALL                    = 2     #所有相机
MODE_VIEW                   = 0     #预览模式
MODE_CAPTURE                = 1  #采集点云模式
MODE_EXTERNAL               = 2  #硬件触发采集点云模式(仅支持线扫模式)
OUTPUT_MODE_ALL             = 0  #全量输出
OUTPUT_MODE_POINT           = 1  #输出点云
OUTPUT_MODE_POINT_AND_DEPTH = 2  #输出点云和深度图
OUTPUT_MODE_POINT_AND_Gray  = 3  #输出点云和灰度图
GRAY_TYPR_UNCORRECT         = 0  #原始图
GRAY_TYPR_CORRECT           = 1  #矫正图
POINT_TYPR_DISORDER         = 0  #无序点云
POINT_TYPR_ORDER            = 1  #有序点云
MODE_BRIGHT                 = 0  #亮场
MODE_DARK                   = 1  #暗场
MODE_MIX                    = 2  #混合场
MODE_NOVICE                 = 0  #新手
MODE_EXPERT                 = 1  #专家
MODE_AREA                   = 0     #面模式
MODE_LINE                   = 1  #线模式
ALGO_NORMAL                 = 0  #常规算法
ALGO_ARC                    = 1     #抗弧光算法
INTERVAL_500                = 0  #500ms
INTERVAL_1000               = 1  #1000ms
INTERVAL_1500               = 2  #1500ms
INTERVAL_2000               = 3  #2000ms
INTERVAL_2500               = 4  #2500ms
INTERVAL_3000               = 5  #3000ms
RECORD_CLOSE                = 0  #无点云录制
RECORD_DIS                  = 1  #以固定距离点云录制
RECORD_SPEED                = 2  #以移动速度点云录制
EXT_TRIG_MODE_EXTERNAL      = 0  #外触发模式
EXT_TRIG_MODE_COUNTER       = 1  #计数器模式
EXT_TRIG_TYPE_LEADING_EDGE  = 0  #上升沿触发
EXT_TRIG_TYPE_TRAILING_EDGE = 1  #下降沿触发

class VSStructure(Structure):
    def __str__(self):
        strs = []
        for field in self._fields_:
            name = field[0]
            value = getattr(self, name)
            if isinstance(value, type(b'')):
                value = _string_buffer_to_str(value)
            strs.append("{}:{}".format(name, value))
        return '\n'.join(strs)
    
    def __repr__(self):
        return self.__str__()
    
    def clone(self):
        obj = type(self)()
        memmove(byref(obj), byref(self), sizeof(self))
        return obj

#相机分辨率
class VSensorImageResolution(VSStructure):
    _fields_ = [
        ("Width", c_int),
        ("Height", c_int)
    ]

#点云重建结果
class VSensorResult(VSStructure):
    _fields_ = [
        ("pointx", c_void_p),
        ("pointy", c_void_p),
        ("pointz", c_void_p),
        ("pointr", c_void_p),
        ("pointg", c_void_p),
        ("pointb", c_void_p),
        ("mask", c_void_p),
        ("DepthMap", c_void_p),
        ("GraySrcMap", c_void_p),
        ("GrayCorMap", c_void_p),
        ("RGBMap", c_void_p),
        ("nPointNum", c_int),
        ("nFrameNum", c_int),
        ("ImageResolution", VSensorImageResolution),
        ("ImageResolutionRgb", VSensorImageResolution),
        ("uTimeStamp", c_long),
        ("posx", c_void_p),
        ("posy", c_void_p),
        ("iRestructionMode", c_int)
    ]

#相机信息
class VSensorCameraInfo(VSStructure):
    _fields_ = [("CameraName", c_char * 32),
                ("Address", c_char * 32),
                ("uInstance", c_uint)]
    
    def GetCameraName(self):
        return _string_buffer_to_str(self.CameraName)
    
    def GetAddress(self):
        return _string_buffer_to_str(self.Address)

#相机内参
class VSensorCameraInternelPara(VSStructure):
    _fields_ = [
        ("LPara", c_double * 9),
        ("RPara", c_double * 9)
    ]

#点云信息
class Array3(ctypes.Structure):
    _fields_ = [("data", ctypes.c_float * 3)]

#线程局部存储
_tls = local()

#存储最后一次SDK调用返回的错误码
def GetLastError():
    try:
        return _tls.last_error
    except AttributeError as e:
        _tls.last_error = 0
        return 0

#设置返回的错误码
def SetLastError(err_code):
    _tls.last_error = err_code

#字符转换
def _string_buffer_to_str(buf):
    s = buf if isinstance(buf, type(b'')) else buf.value
    # Linux环境下优先使用utf-8编码
    for codec in ('utf-8', 'gbk'):
        try:
            s = s.decode(codec)
            break
        except UnicodeDecodeError as e:
            continue

    if isinstance(s, str):
        return s
    else:
        return s.encode('utf-8')

#字符转换
def _str_to_string_buffer(str):
    # Linux环境下使用utf-8编码
    if type(str) is type(u''):
        s = str.encode('utf-8')
    else:
        s = str
    return create_string_buffer(s)

#获取SDK版本号
def GetSdkVersionString():
    pVersionString = create_string_buffer(32)
    err_code = _sdk.GetSdkVersionString(pVersionString)
    SetLastError(err_code)
    return _string_buffer_to_str(pVersionString)

#获取相机列表
def GetDeviceList():
    Nums = c_int(10)
    pCameraList = (VSensorCameraInfo * Nums.value)()
    err_code = _sdk.GetDeviceList(pCameraList, byref(Nums))
    SetLastError(err_code)
    return pCameraList[0:Nums.value]

#获取设备网络地址
def GetDeviceIP(deviceName):
    pAddrString = create_string_buffer(32)
    pMaskString = create_string_buffer(32)
    pGateWayString = create_string_buffer(32)
    pEtAddrString = create_string_buffer(32)
    pEtMaskString = create_string_buffer(32)
    pEtGateWayString = create_string_buffer(32)
    err_code = _sdk.GetDeviceIP(_str_to_string_buffer(deviceName), pAddrString, pMaskString, pGateWayString, pEtAddrString, pEtMaskString, pEtGateWayString)
    SetLastError(err_code)
    return (_string_buffer_to_str(pAddrString), _string_buffer_to_str(pMaskString), _string_buffer_to_str(pGateWayString), _string_buffer_to_str(pEtAddrString), _string_buffer_to_str(pEtMaskString), _string_buffer_to_str(pEtGateWayString))

#设置设备网络地址
def SetDeviceIP(deviceName, addr, mask, gateWay, bPersistent):
    err_code = _sdk.SetDeviceIP(_str_to_string_buffer(deviceName), _str_to_string_buffer(addr), _str_to_string_buffer(mask),_str_to_string_buffer(gateWay), bPersistent)
    return err_code

#配置系统选项
def SetDeviceSysOption(optionName, value):
    err_code = _sdk.SetDeviceSysOption(_str_to_string_buffer(optionName), _str_to_string_buffer(value))
    return err_code

#设备连接
def DeviceConnect(iDeviceIndex):
    err_code = _sdk.DeviceConnect(iDeviceIndex)
    return err_code

#设备参数初始化
def DeviceParameterInit():
    err_code = _sdk.DeviceParameterInit()
    return err_code

#获取设备参数初始化时间
def GetDeviceParameterInitDelayTime():
    time = c_int()
    time1 = byref(time)
    err_code = _sdk.GetDeviceParameterInitDelayTime(time1)
    SetLastError(err_code)
    return time.value

#获取设备自定义序列号
def GetDeviceCustomSN():
    pSerialNumberString = create_string_buffer(32)
    err_code = _sdk.GetDeviceCustomSN(pSerialNumberString)
    SetLastError(err_code)
    return _string_buffer_to_str(pSerialNumberString)

#设置设备自定义序列号
def SetDeviceCustomSN(serialNumber):
    err_code = _sdk.SetDeviceCustomSN(_str_to_string_buffer(serialNumber))
    return err_code

#获取相机内参
def GetCamInternelParameter(pCameraInternelPara):
    err_code = _sdk.GetCamInternelParameter(byref(pCameraInternelPara))
    SetLastError(err_code)
    return pCameraInternelPara

#获取标定参数
def GetPara(para):
    iNum = c_int()
    iNum1 = byref(iNum)
    err_code = _sdk.GetPara(para, iNum1)
    SetLastError(err_code)
    return iNum.value

#获取用户设置模式
def GetUserSettingMode():
    mode = c_int()
    mode1 = byref(mode)
    err_code = _sdk.GetUserSettingMode(mode1)
    SetLastError(err_code)
    return mode.value

#设置用户设置模式
def SetUserSettingMode(mode):
    err_code = _sdk.SetUserSettingMode(c_int(mode))
    return err_code

#重置用户设置数据
def ResetUserSettingData():
    err_code = _sdk.ResetUserSettingData()
    return err_code

#加载用户设置数据
def LoadUserSettingData(filePath):
    err_code = _sdk.LoadUserSettingData(_str_to_string_buffer(filePath))
    return err_code    

#保存用户设置数据
def SaveUserSettingData(filePath):
    err_code = _sdk.SaveUserSettingData(_str_to_string_buffer(filePath))
    return err_code 

#设置当前Z轴范围
def SetZaxisRange(min, max):
    err_code = _sdk.SetZaxisRange(min, max)
    return err_code

#获取当前Z轴范围
def GetZaxisRange():
    iMin = c_int()
    iMin1 = byref(iMin)
    iMax = c_int()
    iMax1 = byref(iMax)
    err_code = _sdk.GetZaxisRange(iMin1, iMax1)
    SetLastError(err_code)
    return (iMin.value, iMax.value)

#设置投影亮度
def SetProjectLight(light):
    err_code = _sdk.SetProjectLight(light)
    return err_code

#设置降采样
def SetDownsampling(isOpen):
    err_code = _sdk.SetDownsampling(isOpen)
    return err_code

#获取曝光时间
def GetExposureTime(flag = TYPE_ALL):
    ExposureTime = c_double()
    ExposureTime1 = byref(ExposureTime)
    err_code = _sdk.GetExposureTime(ExposureTime1, flag)
    SetLastError(err_code)
    return ExposureTime.value

#设置曝光时间
def SetExposureTime(exposureTime, flag = TYPE_ALL):
    err_code = _sdk.SetExposureTime(c_double(exposureTime), flag)
    return err_code

#设置自动曝光
def SetAutoExposureTime(isOpen, targetLight):
    err_code = _sdk.SetAutoExposureTime(isOpen, targetLight)
    return err_code

#设置高动态采集模式        
def SetHDR(exposureTime1, exposureTime2, isOpen):
    err_code = _sdk.SetHDR(c_double(exposureTime1), c_double(exposureTime2), isOpen)
    return err_code

#获取高动态采集模式曝光参数
def GetHDR():
    exposureTime1 = c_double()
    exposureTime1_ = byref(exposureTime1)
    exposureTime2 = c_double()
    exposureTime2_ = byref(exposureTime2)
    err_code = _sdk.GetHDR(exposureTime1_, exposureTime2_)
    SetLastError(err_code)
    return (exposureTime1.value, exposureTime2.value)

#设置自动高动态采集模式
def SetAutoHDR(isOpen, model, targetLight):
    err_code = _sdk.SetAutoHDR(isOpen, model, targetLight)
    return err_code

#设置一键白平衡(此接口仅支持PDR系列和多线扫描彩色3D设备)
def SetCameraOnceWB():
    err_code = _sdk.SetCameraOnceWB()
    return err_code

#设置图像模拟增益值(此接口仅支持PDR系列和多线扫描彩色3D设备)
def SetAnalogGain1(iAnalogGainR, iAnalogGainG, iAnalogGainB):
    err_code = _sdk.SetAnalogGain1(iAnalogGainR, iAnalogGainG, iAnalogGainB)
    return err_code

#获取图像模拟增益值(此接口仅支持PDR系列和多线扫描彩色3D设备)
def GetAnalogGain1():
    iAnalogGainR = c_int()
    iAnalogGainR1 = byref(iAnalogGainR)
    iAnalogGainG = c_int()
    iAnalogGainG1 = byref(iAnalogGainG)
    iAnalogGainB = c_int()
    iAnalogGainB1 = byref(iAnalogGainB)
    err_code = _sdk.GetAnalogGain1(iAnalogGainR1, iAnalogGainG1, iAnalogGainB1)
    SetLastError(err_code)
    return (iAnalogGainR.value, iAnalogGainG.value, iAnalogGainB.value)

#设置图像模拟增益值
def SetAnalogGain2(iAnalogGain, flag = TYPE_ALL):
    err_code = _sdk.SetAnalogGain2(iAnalogGain, flag)
    return err_code

#获取图像模拟增益值
def GetAnalogGain2(flag = TYPE_ALL):
    iAnalogGain = c_int()
    iAnalogGain1 = byref(iAnalogGain)
    err_code = _sdk.GetAnalogGain2(iAnalogGain1, flag)
    SetLastError(err_code)
    return iAnalogGain.value

#获取输出图像分辨率
def GetImageResolution(pImageResolution):
    err_code = _sdk.GetImageResolution(byref(pImageResolution))
    SetLastError(err_code)
    return pImageResolution

#获取输出图像分辨率
def GetImageResolution2(pImageResolutionGray, pImageResolutionRgb):
    err_code = _sdk.GetImageResolution2(byref(pImageResolutionGray), byref(pImageResolutionRgb))
    SetLastError(err_code)
    return (pImageResolutionGray, pImageResolutionRgb)

#获取传感器温度 
def GetDevTemperature():
    devTemperature = c_float()
    devTemperature1 = byref(devTemperature)
    err_code = _sdk.GetDevTemperature(devTemperature1)
    SetLastError(err_code)
    return devTemperature.value   

#复位图像采集时间戳
def ResetFrameTimeStamp():
    err_code = _sdk.ResetFrameTimeStamp()
    return err_code

#获取环境亮度(此接口已废弃)
def GetLuminance1():
    luminance = c_int()
    luminance1 = byref(luminance)
    err_code = _sdk.GetLuminance1(luminance1)
    SetLastError(err_code)
    return luminance.value

#获取环境亮度(此接口已废弃)
def GetLuminance2():
    luminance = c_int()
    luminance1 = byref(luminance)
    err_code = _sdk.GetLuminance2(luminance1)
    SetLastError(err_code)
    return luminance.value

#设置扫描时间间隔(此接口仅支持多线扫描3D设备)
def SetScanTimeInterval(level):
    err_code = _sdk.SetScanTimeInterval(level)
    return err_code

#设置线激光悬停位置(此接口仅支持多线扫描3D设备)
def SetLaserHoverPosition(pos):
    err_code = _sdk.SetLaserHoverPosition(pos)
    return err_code

#设置算法模式(此接口仅支持线面一体和多线扫描3D设备)
def SetAlgoMode(flag):
    err_code = _sdk.SetAlgoMode(flag)
    return err_code

#设置数据补偿
def SetHoleFilling(model, isOpen):
    err_code = _sdk.SetHoleFilling(model, isOpen)
    return err_code

#设置离散点
def SetOutlierDetect(window, para, isOpen):
    err_code = _sdk.SetOutlierDetect(window, c_float(para), isOpen)
    return err_code

#设置离散点2
def SetOutlierDetect2(window, para, isOpen):
    err_code = _sdk.SetOutlierDetect2(window, c_float(para), isOpen)
    return err_code

#设置滤波1
def SetFilter1(window, para, isOpen):
    err_code = _sdk.SetFilter1(window, c_float(para), isOpen)
    return err_code

#设置滤波2
def SetFilter2(para, isOpen):
    err_code = _sdk.SetFilter2(c_float(para), isOpen)
    return err_code

#设置滤波3
def SetFilter3(para, isOpen):
    err_code = _sdk.SetFilter3(c_float(para), isOpen)
    return err_code

#设置滤波4
def SetFilter4(iCutLevel, isOpen):
    err_code = _sdk.SetFilter4(iCutLevel, isOpen)
    return err_code

#设置滤波5
def SetFilter5(para, isOpen):
    err_code = _sdk.SetFilter5(para, isOpen)
    return err_code

#设置ROI感兴趣区域
def SetROI(offsetX, offSetY, width, height, isOpen):
    err_code = _sdk.SetROI(offsetX, offSetY, width, height, isOpen)
    return err_code

#设置补光灯(此接口仅支持加装补光灯的设备)
def SetLight(model):
    err_code = _sdk.SetLight(model)
    return err_code

#设置相机采集模式
def SetCaptureMode(flag):
    err_code = _sdk.SetCaptureMode(flag)
    return err_code  

#设置相机重建模式(此接口仅支持线面一体和多线扫描3D设备切换后需要重新设备参数初始化)
def SetRestructionMode(flag):
    err_code = _sdk.SetRestructionMode(flag)
    return err_code 

#单次重建3D点云
def SingleRestruction(pVSensorResult, flag):
    err_code = _sdk.SingleRestruction(byref(pVSensorResult), flag)
    SetLastError(err_code)
    return pVSensorResult

#开始3D重建连续模式
def ContinuousRestructionStart(flag):
    err_code = _sdk.ContinuousRestructionStart(flag)
    return err_code

#结束3D重建连续模式
def ContinuousRestructionStop():
    err_code = _sdk.ContinuousRestructionStop()
    return err_code

#获取一帧点云数据
def CaptureFrame(pVSensorResult, ctime):
    err_code = _sdk.CaptureFrame(byref(pVSensorResult), ctime)
    SetLastError(err_code)
    return pVSensorResult

#设置外部触发模式
def SetExternalTriggerMode(flag):
    err_code = _sdk.SetExternalTriggerMode(flag)
    return err_code

#设置外部触发延时
def SetExternalTriggerDelay(delayTime):
    err_code = _sdk.SetExternalTriggerDelay(delayTime)
    return err_code

#设置外部触发种类
def SetExternalTriggerType(flag):
    err_code = _sdk.SetExternalTriggerType(flag)
    return err_code

#设置外部触发计数器触发数量设定值
def SetExternalTriggerCounterTriggers(value):
    err_code = _sdk.SetExternalTriggerCounterTriggers(value)
    return err_code

#获取外部触发计数器实时触发数量
def GetExternalTriggerCurrentCounterTriggers():
    count = c_int()
    count1 = byref(count)
    err_code = _sdk.GetExternalTriggerCurrentCounterTriggers(count1)
    SetLastError(err_code)
    return count.value

#复位外部触发计数器
def ResetExternalTriggerCounterTriggers():
    err_code = _sdk.ResetExternalTriggerCounterTriggers()
    return err_code

#设置点云缓存区数量
def SetCloudBufferNum(num):
    err_code = _sdk.SetCloudBufferNum(num)
    return err_code 

#清空点云缓存区
def ClearCloudBuffer():
    err_code = _sdk.ClearCloudBuffer()
    return err_code

#获取缓存区点云
def GetCloudBuffer(num):
    Nums = c_int(num)
    pRestult = (VSensorResult * Nums.value)()
    err_code = _sdk.GetCloudBuffer(pRestult, byref(Nums))
    SetLastError(err_code)
    return pRestult[0:Nums.value]

#录制点云
def Record3DCloud(filePath, flag, para):
    err_code = _sdk.Record3DCloud(_str_to_string_buffer(filePath), flag, c_float(para))
    return err_code

#获取录制点云
def GetRecord3DCloud():
    array_ptr = ctypes.POINTER(Array3)()
    iNum = c_int()
    iNum1 = byref(iNum)
    err_code = _sdk.GetRecord3DCloud(ctypes.byref(array_ptr), iNum1)
    SetLastError(err_code)
    return (array_ptr, iNum.value)

#保存灰度图
def SaveGrayMap(filePath, pVSensorResult, flag):
    err_code = _sdk.SaveGrayMap(_str_to_string_buffer(filePath), byref(pVSensorResult), flag)
    return err_code

#保存深度图
def SaveDepthMap(filePath, pVSensorResult):
    err_code = _sdk.SaveDepthMap(_str_to_string_buffer(filePath), byref(pVSensorResult))
    return err_code

#保存点云
def Save3DCloud(filePath, pVSensorResult, flag):
    err_code = _sdk.Save3DCloud(_str_to_string_buffer(filePath), byref(pVSensorResult), flag)
    return err_code

#保存彩色图(此接口仅支持PDR系列和多线扫描彩色3D设备)
def SaveRGBMap(filePath, pVSensorResult):
    err_code = _sdk.SaveRGBMap(_str_to_string_buffer(filePath), byref(pVSensorResult))
    return err_code

#获取相机采集
def CameraOut1(imgBufferGray, imgBufferRGB):
    err_code = _sdk.CameraOut1(imgBufferGray, imgBufferRGB)
    return err_code

#获取相机采集(此接口已废弃)
def CameraOut2(imgBufferRGB):
    err_code = _sdk.CameraOut2(imgBufferRGB)
    return err_code

#注册获取重建结果回调函数
def RegisterCallbackFuncGetResult(pCallBack):
    err_code = _sdk.RegisterCallbackFuncGetResult(pCallBack)
    return err_code

#设备反初始化
def DeviceUnInit():
    err_code = _sdk.DeviceUnInit()
    return err_code