"""PC/RK standard-library client. Commands are simulation requests only."""
import argparse
import json
import time
import urllib.request
import uuid


class DecisionClient:
    def __init__(self, url, token=None, timeout=2):
        self.url = url.rstrip("/")
        self.token, self.timeout = token, timeout
        self.client_id = "pc_"+uuid.uuid4().hex[:16]
        self.seq = 0

    def snapshot(self):
        with urllib.request.urlopen(self.url+"/snapshot", timeout=self.timeout) as response:
            return json.load(response)

    def prepare(self, action, scenario=None, lane=None):
        snapshot = self.snapshot()
        self.seq += 1
        command = {"client_id": self.client_id, "request_id": uuid.uuid4().hex,
                   "seq": self.seq, "source_epoch": snapshot["source_epoch"], "action": action}
        if action == "set_context":
            command.update(scenario=scenario, lane=lane)
        return command

    def send(self, command):
        # For a transport retry, pass the SAME prepared command: don't manufacture
        # a new request_id or silently retarget an old click after a source change.
        headers = {"Content-Type": "application/json"}
        if self.token:
            headers["X-Decision-Token"] = self.token
        req = urllib.request.Request(self.url+"/command", data=json.dumps(command).encode(), headers=headers, method="POST")
        with urllib.request.urlopen(req, timeout=self.timeout) as response:
            return json.load(response)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--url", default="http://192.168.2.3:18088")
    parser.add_argument("--token")
    parser.add_argument("action", choices=("status", "move", "stop", "left", "right", "clear_alarm", "set_context"))
    parser.add_argument("--scenario", choices=("intersection", "lane_change"), default="intersection")
    parser.add_argument("--lane", choices=("left", "straight", "right"), default="straight")
    args = parser.parse_args()
    client = DecisionClient(args.url, args.token)
    value = client.snapshot() if args.action == "status" else client.send(client.prepare(args.action, args.scenario, args.lane))
    print(json.dumps(value, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
