import os
import sys
import termios
import tty


class TerminalKeyboard:
    def __init__(self, logger):
        self._logger = logger
        self._orig_term = None
        self._configure_terminal()

    def close(self):
        self._restore_terminal()

    def _configure_terminal(self):
        # Active lecture non bloquante en mode cbreak.
        if not sys.stdin.isatty():
            self._logger.warn('stdin is not a TTY; keyboard input disabled')
            return

        self._orig_term = termios.tcgetattr(sys.stdin.fileno())
        tty.setcbreak(sys.stdin.fileno())

    def _restore_terminal(self):
        # Restaure l'etat original du terminal.
        if self._orig_term is not None:
            termios.tcsetattr(sys.stdin.fileno(), termios.TCSANOW, self._orig_term)

    def read_key(self):
        # Lecture non bloquante d'une touche.
        if not sys.stdin.isatty():
            return ''

        import select
        if not select.select([sys.stdin], [], [], 0)[0]:
            return ''

        return os.read(sys.stdin.fileno(), 1).decode(errors='ignore')
