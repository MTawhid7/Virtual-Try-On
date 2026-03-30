import csv

with open('debug/diagnostics.csv', 'r') as f:
    reader = csv.reader(f)
    headers = next(reader)
    rows = list(reader)

impact_idx = None
for i, row in enumerate(rows):
    if float(row[7]) > 10:  # max_grad
        impact_idx = i
        break

if impact_idx is not None:
    print(", ".join(headers))
    start = max(0, impact_idx - 5)
    end = min(len(rows), impact_idx + 20)
    for i in range(start, end):
        print(", ".join(rows[i]))
else:
    print("No impact detected")
