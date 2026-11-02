"""Stand-in for serial.tools.list_ports: tests assign COMPORTS to whatever they want listed."""
COMPORTS = []


def comports(include_links=False):
    return list(COMPORTS)
