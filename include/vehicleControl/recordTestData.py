import csv

class recordTestData(object):
    """Class to print desired information"""

    def __init__(self, gps, state, input, savefile="test"):
        '''Sets up the csv writer and header'''
        data = self.merge(gps, state, input)
        csv_columns = list(data.keys())
        csv_file = "/home/laptopuser/mkz/data/tests/" + savefile + ".csv"
        self.csvfile = open(csv_file,'w')
        self.writer = csv.DictWriter(self.csvfile,
                                fieldnames=csv_columns,
                                quoting=csv.QUOTE_NONNUMERIC)
        self.writer.writeheader()

    def merge(self, gps, state, input):
        '''Merges the dictionarys together'''
        data = {}
        data.update(gps)
        data.update(state)
        data.update(input)
        return data

    def write(self, gps, state, input):
        '''Merges the different dictionarys and prints the data'''
        data = self.merge(gps, state, input)
        self.writer.writerow(data)

    #def __del__(self):
    #    self.csvfile.close()
