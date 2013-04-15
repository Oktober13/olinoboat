import csv

with open('example.csv', 'wb') as csvfile:
	writeme = csv.writer(csvfile)
	writeme.writerow([1,5, 12345.12345, 23456.23456])
	writeme.writerow([1, 3,12346.12345, 23457.23456])
	writeme.writerow([1, 1, 12347.12345,  23458.23456])
