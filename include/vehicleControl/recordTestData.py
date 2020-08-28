import csv

class recordTestData(object):
    """docstring for ."""

    def __init__(self, gps, state, input, savefile="test"):
        data = {}
        data.update(gps)
        data.update(state)
        data.update(input)
        csv_columns = list(data.keys())
        csv_file = "/home/laptopuser/mkz/data/tests/" + savefile + ".csv"
        self.csvfile = open(csv_file,'w')
        self.writer = csv.DictWriter(self.csvfile,
                                fieldnames=csv_columns,
                                quoting=csv.QUOTE_NONNUMERIC)
        self.writer.writeheader()

    def write(self, gps, state, input):
        data = {}
        data.update(gps)
        data.update(state)
        data.update(input)
        self.writer.writerow(data)

    def __del__(self):
        close(self.csvfile)
