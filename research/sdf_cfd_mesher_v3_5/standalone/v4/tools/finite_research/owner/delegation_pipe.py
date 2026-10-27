import ctypes as native
from ctypes import wintypes
import json,time,uuid

class Pipe:
    def __init__(self,address=None):
        self.kernel=native.WinDLL('kernel32',use_last_error=True)
        self.server=address is None
        self.address=address or '\\\\.\\pipe\\v4-delegate-'+uuid.uuid4().hex
        self.handle=None
        self.connected=not self.server
        signatures={
            'CreateNamedPipeW':([wintypes.LPCWSTR,wintypes.DWORD,wintypes.DWORD,wintypes.DWORD,wintypes.DWORD,wintypes.DWORD,wintypes.DWORD,native.c_void_p],wintypes.HANDLE),
            'CreateFileW':([wintypes.LPCWSTR,wintypes.DWORD,wintypes.DWORD,native.c_void_p,wintypes.DWORD,wintypes.DWORD,wintypes.HANDLE],wintypes.HANDLE),
            'ConnectNamedPipe':([wintypes.HANDLE,native.c_void_p],wintypes.BOOL),
            'SetNamedPipeHandleState':([wintypes.HANDLE,native.POINTER(wintypes.DWORD),native.c_void_p,native.c_void_p],wintypes.BOOL),
            'GetNamedPipeClientProcessId':([wintypes.HANDLE,native.POINTER(wintypes.ULONG)],wintypes.BOOL),
            'GetNamedPipeServerProcessId':([wintypes.HANDLE,native.POINTER(wintypes.ULONG)],wintypes.BOOL),
            'ReadFile':([wintypes.HANDLE,native.c_void_p,wintypes.DWORD,native.POINTER(wintypes.DWORD),native.c_void_p],wintypes.BOOL),
            'WriteFile':([wintypes.HANDLE,native.c_void_p,wintypes.DWORD,native.POINTER(wintypes.DWORD),native.c_void_p],wintypes.BOOL),
            'CloseHandle':([wintypes.HANDLE],wintypes.BOOL)}
        for name,(arguments,result) in signatures.items():
            function=getattr(self.kernel,name)
            function.argtypes=arguments
            function.restype=result
        if self.server:
            self.handle=self.kernel.CreateNamedPipeW(self.address,3|0x80000,4|2|1|8,1,65536,65536,0,None)
            if self.handle==wintypes.HANDLE(-1).value:
                self.handle=None
                raise native.WinError(native.get_last_error())

    def connect(self,deadline):
        while time.perf_counter()<deadline:
            handle=self.kernel.CreateFileW(self.address,0xc0000000,0,None,3,0,None)
            if handle!=wintypes.HANDLE(-1).value:
                self.handle=handle
                mode=wintypes.DWORD(2|1)
                if not self.kernel.SetNamedPipeHandleState(handle,native.byref(mode),None,None):
                    self.close()
                    raise native.WinError(native.get_last_error())
                return
            error=native.get_last_error()
            if error not in (2,231):
                raise native.WinError(error)
            time.sleep(.025)
        raise TimeoutError('DelegationPipeConnectionDeadline')

    def peer_pid(self):
        result=wintypes.ULONG()
        operation=self.kernel.GetNamedPipeClientProcessId if self.server else self.kernel.GetNamedPipeServerProcessId
        if not operation(self.handle,native.byref(result)):
            raise native.WinError(native.get_last_error())
        return result.value

    def receive(self):
        if self.server and not self.connected:
            if self.kernel.ConnectNamedPipe(self.handle,None):
                return None
            else:
                error=native.get_last_error()
                if error==536:
                    return None
                if error!=535:
                    raise native.WinError(error)
            self.connected=True
        buffer=native.create_string_buffer(65536)
        count=wintypes.DWORD()
        if not self.kernel.ReadFile(self.handle,buffer,65536,native.byref(count),None):
            error=native.get_last_error()
            if error==232:
                return None
            raise native.WinError(error)
        if not count.value:
            raise ValueError('EmptyDelegationPipeMessage')
        return json.loads(buffer.raw[:count.value])

    def send(self,value):
        content=json.dumps(value,sort_keys=True,allow_nan=False).encode()
        if len(content)>60000:
            raise ValueError('DelegationPipeMessageTooLarge')
        count=wintypes.DWORD()
        if not self.kernel.WriteFile(self.handle,content,len(content),native.byref(count),None):
            raise native.WinError(native.get_last_error())
        if count.value!=len(content):
            raise ValueError('IncompleteDelegationPipeWrite')

    def close(self):
        if self.handle is not None:
            if not self.kernel.CloseHandle(self.handle):
                raise native.WinError(native.get_last_error())
            self.handle=None
