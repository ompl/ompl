import pytest
import tempfile
import os
from ompl import util

def test_set_get_log_level():
    # Save original log level
    original_level = util.getLogLevel()

    # Test setting each log level
    for level in [util.LogLevel.LOG_DEBUG, util.LogLevel.LOG_INFO,
                  util.LogLevel.LOG_WARN, util.LogLevel.LOG_ERROR,
                  util.LogLevel.LOG_NONE]:
        util.setLogLevel(level)
        assert util.getLogLevel() == level

    # Restore original log level
    util.setLogLevel(original_level)


def test_logging_functions():
    # These should not raise exceptions
    util.OMPL_ERROR("Test error message")
    util.OMPL_WARN("Test warning message")
    util.OMPL_INFORM("Test info message")
    util.OMPL_DEBUG("Test debug message")
    util.OMPL_DEVMSG1("Test dev1 message")
    util.OMPL_DEVMSG2("Test dev2 message")


def test_output_handler_std():
    # Create an OutputHandlerSTD and verify it can be used
    handler = util.OutputHandlerSTD()
    assert handler is not None

    # Test logging through the handler directly
    handler.log("Direct log test", util.LogLevel.LOG_INFO, "test_console.py", 42)


def test_output_handler_file():
    # Create a temporary file for logging
    with tempfile.NamedTemporaryFile(mode='w', delete=False, suffix='.log') as f:
        temp_path = f.name

    try:
        # Create file handler and log a message
        file_handler = util.OutputHandlerFile(temp_path)
        file_handler.log("File log test", util.LogLevel.LOG_INFO, "test_console.py", 50)

        # Clean up the handler to flush/close the file
        del file_handler

        # Verify the file was created and contains content
        assert os.path.exists(temp_path)
        with open(temp_path, 'r') as f:
            content = f.read()
            assert "File log test" in content
    finally:
        # Clean up
        if os.path.exists(temp_path):
            os.remove(temp_path)


def test_use_output_handler():
    # Get original handler
    original_handler = util.getOutputHandler()

    # Create and set a new handler
    new_handler = util.OutputHandlerSTD()
    util.useOutputHandler(new_handler)

    # Verify we can get the handler back
    current_handler = util.getOutputHandler()
    assert current_handler is not None

    # Restore original handler
    util.restorePreviousOutputHandler()


def test_no_output_handler():
    # Get original handler
    original_handler = util.getOutputHandler()

    # Disable output
    util.noOutputHandler()
    assert util.getOutputHandler() is None

    # Log messages should not raise exceptions even with no handler
    util.OMPL_INFORM("This should not appear")

    # Restore previous handler
    util.restorePreviousOutputHandler()


def test_log_level_filtering():
    # Save original log level
    original_level = util.getLogLevel()

    # Set log level to WARN - only WARN and ERROR should be logged
    util.setLogLevel(util.LogLevel.LOG_WARN)

    # These calls should not raise exceptions
    util.OMPL_DEBUG("Debug - should be filtered")
    util.OMPL_INFORM("Info - should be filtered")
    util.OMPL_WARN("Warning - should appear")
    util.OMPL_ERROR("Error - should appear")

    # Set to NONE - nothing should be logged
    util.setLogLevel(util.LogLevel.LOG_NONE)
    util.OMPL_ERROR("This error should be filtered")

    # Restore original log level
    util.setLogLevel(original_level)

