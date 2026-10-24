#!/usr/bin/env python3
"""Convert supplied crop-row JSON into the accuracy analyzer's CSV schema."""
import argparse
import csv
import json
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('source', type=Path)
    parser.add_argument('output', type=Path)
    args = parser.parse_args()
    data = json.loads(args.source.read_text())
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open('w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['row_id', 'x1', 'y1', 'x2', 'y2'])
        for plot_id, plot in data.items():
            for index, (start, end) in enumerate(plot['centerlines']):
                writer.writerow([f'{plot_id}_{index}', *start, *end])
    print(f'Exported {args.output}')

if __name__ == '__main__':
    main()
