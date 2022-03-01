import signal
from datetime import datetime
from multiprocessing import Queue, Process
from time import sleep


class Logger:
    logThread = None
    shouldThreadJoin = False
    
    buffer = Queue()
    
    logToConsole = True
    logToFile = False
    filepath = ""
    
    @classmethod
    def setLogToConsole(cls, enable: bool):
        """ Tells the Logger whether to print to the console

        Args:
            enable (boolean): Whether to print to the console
        """
        cls.buffer.put(("logToConsole", enable))
    
    @classmethod
    def openFile(cls, filepath: str):
        """ Confirms that the file can be opened and prints an opening message
        
        Args:
            filepath (str): The file that the log should be written to
        """

        if filepath == "":
            cls.buffer.put(("logToFile", False))
            return
        
        try:
            file = open(filepath, "a")
            file.write("\n\nStarting log at [" + datetime.now().strftime("%H:%M:%S") + "]\n")
            file.close()
            cls.buffer.put(("filepath", filepath))
            cls.buffer.put(("logToFile", True))
        except IOError:
            print("ERROR: unable to log to file at: " + filepath)
            cls.buffer.put(("logToFile", False))
    
    @classmethod
    def close(cls):
        """ DO NOT DIRECTLY CALL - If logger is open, output a closing message to the file
        """
        if cls.logToFile:
            try:
                file = open(cls.filepath, "a")
                file.write("Closing log at [" + datetime.now().strftime("%H:%M:%S") + "]\n\n\n")
                file.close()
            except IOError:
                return
    
    @classmethod
    def log(cls, message):
        """ Adds the message to the buffer, then the logger thread logs the provided string to the console and/or file
            as configured

        Args:
            message (str/Exception): The message to be output to the console and/or file
        """
        # enables .log to handle Exception types directly
        if isinstance(message, Exception):
            cls.buffer.put(repr(message))
        elif isinstance(message, str):
            cls.buffer.put(message)
        else:
            cls.buffer.put("ERROR: Cannot log message of type " + str(type(message)))
    
    @classmethod
    def init(cls, filepath=""):
        """ Start the logger thread, will log to file if specified
        """
        cls.buffer = Queue()
        cls.buffer.put(("shouldThreadJoin", False))
        cls.filepath = filepath
        Logger.openFile(filepath)
        cls.logThread = Process(target=Logger.runLogThread, args=(cls.buffer,), daemon=True)
        cls.logThread.start()
        return
    
    @classmethod
    def shutdown(cls):
        """Stops the logger thread
        """
        cls.buffer.put(("shouldThreadJoin", True))
        print(f"LOGGER QUEUE EMPTY: {cls.buffer.empty()}")
        if cls.logThread.is_alive():
            cls.logThread.join()

    @classmethod
    def handle_sigint_signal(cls, sig, frame):
        cls.shouldThreadJoin = True
        cls.buffer.put("Logger received SIGINT signal!")
        return

    @classmethod
    def handle_sigterm_signal(cls, sig, frame):
        cls.shouldThreadJoin = True
        cls.buffer.put("Logger received SIGTERM signal!")
        return
    
    @classmethod
    def runLogThread(cls, buffer: Queue):
        """Function used by the logger thread

        Args:
            buffer (Queue): queue for putting the messages when using .log method
        """
        cls.buffer = buffer

        signal.signal(signal.SIGINT, Logger.handle_sigint_signal)
        signal.signal(signal.SIGTERM, Logger.handle_sigterm_signal)

        while True:
            while not cls.buffer.empty():
                val = cls.buffer.get()
                if isinstance(val, str):
                    finalMessage = f'[{datetime.now().strftime("%H:%M:%S")}] {val}\n'
                    if cls.logToConsole:
                        print(finalMessage, end="")
                    if cls.logToFile:  # and toFile:
                        try:
                            file = open(cls.filepath, "a")
                            file.write(finalMessage)
                            file.close()
                        except IOError:
                            return
                elif isinstance(val, tuple):
                    try:
                        logSetting = True
                        if val[0] == "logToConsole":
                            assert isinstance(val[1], bool)
                            cls.logToConsole = val[1]
                        elif val[0] == "logToFile":
                            assert isinstance(val[1], bool)
                            cls.logToFile = val[1]
                        elif val[0] == "filepath":
                            assert isinstance(val[1], str)
                            cls.filepath = val[1]
                        elif val[0] == "shouldThreadJoin":
                            assert isinstance(val[1], bool)
                            cls.shouldThreadJoin = val[1]
                            logSetting = False
                        else:
                            cls.buffer.put(f'Unknown setting <{val[0]}> with value <{str(val[1])}')
                            logSetting = False
                        if logSetting:
                            cls.buffer.put(f'Logger setting <{val[0]}> is now <{str(val[1])}>')
                    except AssertionError:
                        cls.buffer.put(f'Unknown setting <{str(val[0])}> with value <{str(val[1])}')
                else:
                    cls.buffer.put(f'Invalid logger command of type <{type(val)}>')
            
            if cls.shouldThreadJoin and cls.buffer.empty():
                break
            
            sleep(0.1)
        
        Logger.close()
