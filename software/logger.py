import logging
from datetime import datetime
import os


class TerminalFormatter(logging.Formatter):
    grey = "\x1b[38;20m"
    yellow = "\x1b[33;20m"
    red = "\x1b[31;20m"
    bold_red = "\x1b[31;1m"
    reset = "\x1b[0m"
    format_template = "[%(asctime)s] %(name)s - %(levelname)s - %(message)s (%(filename)s:%(lineno)d)"

    FORMATS = {
        logging.DEBUG: grey + format_template + reset,
        logging.INFO: grey + format_template + reset,
        logging.WARNING: yellow + format_template + reset,
        logging.ERROR: red + format_template + reset,
        logging.CRITICAL: bold_red + format_template + reset
    }

    def format(self, record):
        log_fmt = self.FORMATS.get(record.levelno)
        formatter = logging.Formatter(log_fmt)
        return formatter.format(record)


class Logger():
    def __init__(self):
        # make the logs dir if it doesn't exist already
        if not os.path.exists("logs"):
            os.mkdir("logs")
        # saving log to file
        fh = logging.FileHandler(
            "logs/" + datetime.now().strftime("%Y.%m.%d %H.%M.%S") + ".log")
        # we only want the template from it, not colours
        fh.formatter = logging.Formatter(TerminalFormatter.format_template)
        # displaying log in terminal
        ch = logging.StreamHandler()
        ch.setFormatter(TerminalFormatter())
        self.log = logging.getLogger("%placeholder%")
        self.log.addHandler(fh)
        self.log.addHandler(ch)
        self.log.setLevel(logging.DEBUG)  # TODO - make log level adjustable in main.py


if __name__ == "__main__":
    logger = Logger()

    logger.log.debug('Debug message')
    logger.log.info('Info message')
    logger.log.warning('Warning message')
    logger.log.error('Error message')
