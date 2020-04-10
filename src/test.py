import inspect
import ctypes
import signal
import sys
import time
from threading import Thread


def _async_raise(tid, exctype):
    """raises the exception, performs cleanup if needed"""
    tid = ctypes.c_long(tid)
    if not inspect.isclass(exctype):
        exctype = type(exctype)
    res = ctypes.pythonapi.PyThreadState_SetAsyncExc(tid, ctypes.py_object(exctype))
    if res == 0:
        raise ValueError("invalid thread id")
    elif res != 1:
        # """if it returns a number greater than one, you're in trouble,
        # and you should call it again with exc=NULL to revert the effect"""
        ctypes.pythonapi.PyThreadState_SetAsyncExc(tid, None)
        raise SystemError("PyThreadState_SetAsyncExc failed")


def stop_thread(thread):
    _async_raise(thread.ident, SystemExit)


def signal_handler(signal, frame):
    print('You pressed Ctrl+C!')
    stop_thread(t1)
    stop_thread(t2)
    sys.exit()


def Receive():
    while 1:
        print(1)
        time.sleep(0.1)


def Send():
    while 1:
        print(2)
        time.sleep(0.1)


if __name__ == "__main__":
    t1 = Thread(target=Receive)
    t2 = Thread(target=Send)
    t1.start()
    t2.start()
    print(3)
    while True:
        signal.signal(signal.SIGINT, signal_handler)
        print(4)
        time.sleep(0.1)
