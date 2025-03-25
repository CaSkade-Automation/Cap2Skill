import logging
import colorlog

# Remove existing handlers
for handler in logging.root.handlers[:]:
    logging.root.removeHandler(handler)

# ANSI escape codes for text styling
BOLD = "\033[1m"
UNDERLINE = "\033[4m"
RED = "\033[31m"
RESET = "\033[0m"

# Define log colors
log_colors = {
    "DEBUG": "cyan",
    "INFO": "green",
    "WARNING": "yellow",
    "ERROR": "red",
    "CRITICAL": "bold_red",
}

# Define log formatter
formatter = colorlog.ColoredFormatter(
    "%(blue)s[%(asctime)s]%(reset)s %(purple)s[%(name)s]%(reset)s %(log_color)s[%(levelname)s]:%(reset)s %(message)s",
    log_colors=log_colors,
    reset=True,
)

# Create a console handler with color formatting
console_handler = logging.StreamHandler()
console_handler.setFormatter(formatter)

# Configure logging
logging.basicConfig(level=logging.INFO, handlers=[console_handler])