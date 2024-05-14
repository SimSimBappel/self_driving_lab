import statistics

successful_operations_seconds = [
    118,    # Operation 1 (01:58 -> 1*60 + 58 = 118 seconds)
    120,    # Operation 2 (02:00 -> 2*60 + 0 = 120 seconds)
    135,    # Operation 3 (02:15 -> 2*60 + 15 = 135 seconds)
    126,    # Operation 5 (02:06 -> 2*60 + 6 = 126 seconds)
    150,    # Operation 6 (02:30 -> 2*60 + 30 = 150 seconds)
    130,    # Operation 7 (02:10 -> 2*60 + 10 = 130 seconds)
    195,     # Operation 10 (03:15 -> 3*60 + 15 = 195 seconds)
    135     # Operation 10 (02:15 -> 2*60 + 15 = 135 seconds)
]

std_dev = statistics.stdev(successful_operations_seconds)
print(f"The standard deviation is {std_dev}")