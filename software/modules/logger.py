import logging
from datetime import datetime
import os


class TerminalFormatter(logging.Formatter):
    """Custom formatter for the logger to use in the terminal."""

    # ANSI escape sequences for colors
    grey = "\x1b[38;20m"
    yellow = "\x1b[33;20m"
    red = "\x1b[31;20m"
    bold_red = "\x1b[31;1m"
    reset = "\x1b[0m"

    # Format string for log messages
    format_template = "[%(asctime)s] %(name)s - %(levelname)s - %(message)s (%(filename)s:%(lineno)d)"

    # Mapping of log levels to format strings
    formats = {
        logging.DEBUG: grey + format_template + reset,
        logging.INFO: grey + format_template + reset,
        logging.WARNING: yellow + format_template + reset,
        logging.ERROR: red + format_template + reset,
        logging.CRITICAL: bold_red + format_template + reset
    }

    def format(self, record):
        """Format the log record using the appropriate format string."""
        log_fmt = self.formats.get(record.levelno)
        formatter = logging.Formatter(log_fmt)
        return formatter.format(record)


class Logger:
    """Custom logger for the project."""

    def __init__(self, log_level, name="%placeholder%"):
        # Create the logs directory if it does not already exist
        if not os.path.exists("logs"):
            os.mkdir("logs")

        # Set up file and stream handlers for the logger
        fh = logging.FileHandler(
            "logs/" + datetime.now().strftime("%Y.%m.%d %H.%M.%S") + ".log")
        # Use the TerminalFormatter format string without colors for the file handler
        fh.formatter = logging.Formatter(TerminalFormatter.format_template)
        ch = logging.StreamHandler()
        ch.setFormatter(TerminalFormatter())

        # Initialize the logger with the specified name and handlers
        self.log = logging.getLogger(name)
        self.log.addHandler(fh)
        self.log.addHandler(ch)
        self.log.setLevel(log_level)


if __name__ == "__main__":
    # Test the logger
    logger = Logger()

    logger.log.debug('Debug message')
    logger.log.info('Info message')
    logger.log.warning('Warning message')
    logger.log.error('Error message')
