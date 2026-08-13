import threading
import csv
import time

from HeraSample import make_sample


class CsvReplaySource(threading.Thread):
    def __init__(
        self,
        out_queue,
        filename,
        measure_filter,
        node_filter,
        format_name,
        replay_speed=1.0,
        mode="replay"  # "replay" ou "full"
    ):
        super().__init__(daemon=True)
        self.out_queue = out_queue
        self.filename = filename
        self.measure_filter = measure_filter
        self.node_filter = node_filter
        self.format_name = format_name
        self.replay_speed = replay_speed
        self.mode = mode
        self.running = False

    def stop(self):
        self.running = False

    def run(self):
        self.running = True

        try:
            samples = self.read_all_samples()

            if self.mode == "full":
                self.out_queue.put(("samples", samples))
                return

            for sample in samples:
                if not self.running:
                    break

                self.out_queue.put(("sample", sample))
                time.sleep(0.02 / self.replay_speed)

        except Exception as e:
            self.out_queue.put(("error", f"Error CSV: {e}"))

    def read_all_samples(self):
        samples = []

        with open(self.filename, "r", newline="") as f:
            reader = csv.reader(f)

            for row in reader:
                if not self.running:
                    break

                sample = self.parse_row(row)
                if sample is not None:
                    samples.append(sample)

        return samples

    def parse_row(self, row):
        """
        Format CSV Hera attendu :
            seq, heraTimestamp, values...
        """
        try:
            row = [cell.strip() for cell in row]

            if not row or len(row) < 3:
                return None

            # Ignore header éventuel
            if row[0].lower() in ["seq", "sequence"]:
                return None

            seq = int(float(row[0]))
            hera_timestamp = float(row[1])

            # On enlève seulement seq et heraTimestamp.
            values = [float(x) for x in row[2:] if x != ""]

            return make_sample(
                measure=self.measure_filter,
                node=self.node_filter,
                seq=seq,
                values=values,
                format_name=self.format_name,
                hera_timestamp=hera_timestamp
            )

        except Exception as e:
            print("CSV parse error:", e, "row:", row)
            return None