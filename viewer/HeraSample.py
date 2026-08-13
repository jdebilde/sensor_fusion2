FORMATS = {
    "nav2": {
        "columns": [
            "nav_timestamp",
            "dt",
            "ax",
            "ay",
            "az",
            "gx",
            "gy",
            "gz",
        ]
    },

    "nav1_acc": {
        "columns": [
            "timestamp",
            "dt",
            "ax",
            "ay",
            "az",
        ]
    },

    "nav1_gyro": {
        "columns": [
            "timestamp",
            "dt",
            "gx",
            "gy",
            "gz",
        ]
    },

    "nav1_mag": {
        "columns": [
            "timestamp",
            "dt",
            "mx",
            "my",
            "mz",
        ]
    },

    "nav2_ekf": {
        "columns": [
            "px",
            "py",
            "vx",
            "vy",
            "ax",
            "ay",
            "stopped",
        ]
    },

    "heading_gyro": {
        "columns": [
            "timestamp",
            "dt",
            "roll",
            "pitch",
            "yaw",
        ]
    },

    "ekf_v1": {
        "columns": [
            "timestamp",
            "dt",

            "roll",
            "pitch",
            "yaw",

            "gyro_x",
            "gyro_y",
            "gyro_z",

            "acc_x",
            "acc_y",
            "acc_z",

            "grav_res_x",
            "grav_res_y",
            "grav_res_z",

            "lin_acc_x",
            "lin_acc_y",
            "lin_acc_z",

            "world_acc_x",
            "world_acc_y",
        ]
    },
}


class HeraSample:
    def __init__(self, measure, node, seq, values, fields, hera_timestamp=None):
        self.measure = measure
        self.node = node
        self.seq = int(seq)
        self.values = values
        self.fields = fields
        self.hera_timestamp = hera_timestamp

        for key, value in fields.items():
            setattr(self, key, value)


def make_sample(measure, node, seq, values, format_name, hera_timestamp=None):
    if format_name not in FORMATS:
        raise ValueError(f"Unknown format: {format_name}")

    columns = FORMATS[format_name]["columns"]
    fields = {}

    for index, column in enumerate(columns):
        if index < len(values):
            fields[column] = float(values[index])
        else:
            fields[column] = None

    return HeraSample(
        measure=measure,
        node=node,
        seq=seq,
        values=values,
        fields=fields,
        hera_timestamp=hera_timestamp
    )
