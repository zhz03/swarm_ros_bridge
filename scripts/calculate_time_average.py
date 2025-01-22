import re

def read_numbers_and_calculate_average(file_path):
    # List to store the numbers
    numbers = []
    
    # Regular expression to match the number before "ms"
    # This regex looks for a number (integer or float) followed by optional spaces and 'ms'
    pattern = re.compile(r"([0-9]*\.?[0-9]+)\s*ms")
    
    # Open and read the file line by line
    with open(file_path, 'r') as file:
        for line in file:
            # Search for the pattern in the line
            match = pattern.search(line)
            if match:
                # Convert the captured group to a float and append to the list
                number = float(match.group(1))
                numbers.append(number)
    
    # Calculate the average if the list is not empty
    if numbers:
        average = sum(numbers) / len(numbers)
    else:
        average = 0.0
        
    return numbers, average

if __name__ == '__main__':
    # Specify the path to your txt file
    file_path = './ros_time_cal.txt'
    
    # Get the list of numbers and the average
    numbers, average = read_numbers_and_calculate_average(file_path)
    
    print("Extracted numbers (ms):", numbers)
    print("Average (ms):", average)
