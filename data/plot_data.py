import pandas as pd
import matplotlib.pyplot as plt
import argparse

def main():
    parser = argparse.ArgumentParser(
        description="Plot distance vs. row number from a CSV file."
    )
    parser.add_argument(
        "csv_file", 
        type=str, 
        help="Path to the CSV file with at least two columns."
    )
    args = parser.parse_args()

    # Read the CSV file
    df = pd.read_csv(args.csv_file)

    # Check that there are at least two columns
    if df.shape[1] < 2:
        print("Error: CSV file must have at least two columns.")
        return

    # Identify the column for distance
    # (Assuming the second column in the CSV is the distance)
    distance_col = df.columns[1]

    # Plot the distance against the row index
    plt.figure(figsize=(8, 5))
    plt.plot(range(len(df)), df[distance_col], marker='o', linestyle='-')
    plt.xlabel("Row Number")
    plt.ylabel("Distance")
    plt.title("Distance vs. Row Number")
    plt.grid(True)
    plt.tight_layout()
    plt.show()

if __name__ == '__main__':
    main()


#import pandas as pd
# import matplotlib.pyplot as plt
# import argparse

# def main():
#     # 1. Parse arguments
#     parser = argparse.ArgumentParser(
#         description="Process a CSV file with timestamps and distances, compute statistics, and plot the data."
#     )
#     parser.add_argument(
#         "csv_file", 
#         type=str, 
#         help="Path to the CSV file. The file should have at least two columns: the first as a timestamp and the second as a distance measurement."
#     )
#     args = parser.parse_args()

#     # 2. Read the CSV file
#     try:
#         df = pd.read_csv(args.csv_file)
#     except Exception as e:
#         print(f"Error reading CSV file: {e}")
#         return

#     # 3. Check that there are at least two columns
#     if df.shape[1] < 2:
#         print("Error: CSV file must have at least two columns (timestamp and distance).")
#         return

#     # 4. Identify the columns
#     timestamp_col = df.columns[0]
#     distance_col = df.columns[1]

#     # 5. Convert the first column to datetime
#     try:
#         df[timestamp_col] = pd.to_datetime(df[timestamp_col])
#     except Exception as e:
#         print(f"Error converting {timestamp_col} to datetime: {e}")
#         return

#     # 6. Sort by timestamp
#     df.sort_values(by=timestamp_col, inplace=True)

#     # 7. Compute the average and variance of the distance column
#     avg_distance = df[distance_col].mean()
#     variance_distance = df[distance_col].var()

#     print(f"Average Distance: {avg_distance}")
#     print(f"Variance of Distance: {variance_distance}")

#     # ------------------------------------------------
#     # 8. Group by timestamp to get 1 point per unique timestamp
#     #    (e.g., take the first measurement in each group)
#     # ------------------------------------------------
#     df_unique = df.groupby(timestamp_col, as_index=False)[distance_col].first()

#     # 9. Plot the new dataframe with unique timestamps
#     plt.figure(figsize=(10, 6))
#     plt.plot(df_unique[timestamp_col], df_unique[distance_col], marker='o', linestyle='-')
#     plt.xlabel("Timestamp")
#     plt.ylabel("Distance")
#     plt.title("Distance over Time (Unique Timestamps)")
#     plt.grid(True)
#     plt.tight_layout()
#     plt.show()

# if __name__ == '__main__':
#     main()
