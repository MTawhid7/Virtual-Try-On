import csv
import sys

def analyze():
    print('--- CUSICK DRAPE METRICS ANALYSIS ---')
    try:
        with open('debug/cusick_deep_metrics.csv', 'r') as f:
            reader = csv.DictReader(f)
            data = list(reader)
    except Exception as e:
        print(f"Error: {e}")
        return

    if not data:
        print("Empty data")
        return

    first_nan = None
    for row in data:
        if row['nan_detected'] == '1':
            first_nan = int(row['frame'])
            break

    if first_nan is not None:
        print(f"!!! NaN Detected at Frame {first_nan} !!!")
    else:
        print("No NaNs detected.")

    print("\nTracing key metrics around contact (frames 0 to 150):")
    print(f"{'Frame':<6} | {'Active':<6} | {'Max_Viol':<10} | {'Max_Hess':<11} | {'Min_CCD':<10} | {'Max_Speed':<10} | {'KE':<10}")
    print("-" * 80)
    for row in data[::10]:
        frame = int(row['frame'])
        act = row['active_contacts']
        viol = float(row['max_viol'])
        hess = float(row['max_hess'])
        ccd = float(row['min_ccd_toi'])
        spd = float(row['max_speed'])
        ke = float(row['ke'])
        print(f"{frame:<6} | {act:<6} | {viol:<10.6f} | {hess:<11.4f} | {ccd:<10.6f} | {spd:<10.4f} | {ke:<10.6f}")

    print("\nDiagnostic Summary:")
    max_ke = max(float(r['ke']) for r in data)
    max_spd = max(float(r['max_speed']) for r in data)
    max_viol = max(float(r['max_viol']) for r in data)

    print(f"Peak KE: {max_ke:.6f}")
    print(f"Peak Speed: {max_spd:.4f}")
    print(f"Peak Violation: {max_viol:.6f}")

    # Check if hessian drops to 0 after contact
    contact_start = False
    dropped_hessian = False
    for r in data:
        if int(r['active_contacts']) > 0:
            contact_start = True
        if contact_start and float(r['max_hess']) == 0.0 and int(r['active_contacts']) > 0:
            dropped_hessian = True

    print(f"Hessian suddenly dropped to 0 while in contact? {dropped_hessian}")

if __name__ == '__main__':
    analyze()
