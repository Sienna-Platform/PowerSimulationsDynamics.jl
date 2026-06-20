#!/usr/bin/env python

"""Converts timeseries_pointers between CSV and JSON"""

import json
import os
import sys


CSV_FILE = os.path.join("RTS_GMLC", "timeseries_pointers.csv")
JSON_FILE = os.path.join("RTS_GMLC", "timeseries_pointers.json")


def csv_to_json():
    with open(CSV_FILE) as f_in:
        lines = f_in.readlines()

    assert lines
    header = lines[0].strip()
    names = header.split(",")
    records = []
    for line in lines[1:]:
        line = line.strip()
        record = {}
        for i, val in enumerate(line.split(",")):
            if names[i] == "scaling_factor":
                val = float(val)
            record[names[i]] = val
        records.append(record)

    with open(JSON_FILE, "w") as f_out:
        json.dump(records, f_out, indent=4)
        # Make git happy.
        f_out.write("\n")

    print(f"Converted {CSV_FILE} to {JSON_FILE}")


def json_to_csv():
    with open(JSON_FILE) as f_in:
        data = json.load(f_in)

    assert data
    lines = []
    with open(CSV_FILE, "w") as f_out:
        f_out.write(",".join(data[0].keys()) + "\n")
        for item in data:
            line = ",".join([str(x) for x in item.values()]) + "\n"
            f_out.write(line)

    print(f"Converted {JSON_FILE} to {CSV_FILE}")


def main():
    if len(sys.argv) == 1 or sys.argv[1] == "csv_to_json":
        func = csv_to_json
    elif sys.argv[1] == "json_to_csv":
        func = json_to_csv
    else:
        print(f"{sys.argv[1]} is invalid. Use 'csv_to_json' or 'json_to_csv'.")
        sys.exit(1)

    func()


if __name__ == "__main__":
    main()
