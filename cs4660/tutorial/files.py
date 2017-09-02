"""Files tests simple file read related operations"""

class SimpleFile(object):
    """SimpleFile tests using file read api to do some simple math"""
    def __init__(self, file_path):
        self.numbers = []
        with open(file_path) as file:
            for line in file:
                line = line.strip().split(" ")
                self.numbers.append(list(map(int,line)))

    def get_mean(self, line_number):
        line = self.numbers[line_number]
        return sum(line)/float(len(line))

    def get_max(self, line_number):
        line = self.numbers[line_number]
        line.sort()
        return line[-1]

    def get_min(self, line_number):
        line = self.numbers[line_number]
        line.sort()
        return line[0]

    def get_sum(self, line_number):
        return sum(self.numbers[line_number])