from pathlib import Path
import sys
import threading
from types import SimpleNamespace as NS
import unittest

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'PC_Windows'),str(ROOT/'PC_Windows/Console_Application')]
from navigation_session import publish_view_state


class ViewMailboxTests(unittest.TestCase):
    def test_worker_publishes_owned_snapshot_without_calling_gui(self):
        window=NS()
        state=NS(actual_speed_kmh=10.)
        t=threading.Thread(target=lambda:publish_view_state(window,state))
        t.start();t.join()
        state.actual_speed_kmh=40.
        self.assertEqual(window._navigation_view_pending.actual_speed_kmh,10.)

    def test_many_updates_keep_only_latest_pending_state(self):
        window=NS()
        for n in range(500):
            publish_view_state(window,NS(actual_speed_kmh=n))
        self.assertEqual(window._navigation_view_pending.actual_speed_kmh,499)
