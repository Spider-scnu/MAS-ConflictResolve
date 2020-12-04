import string

row, col = 25, 40

blank_line = "0," * (col - 1) + "0" + "\n"
new_line = ""

text_str = str(row) + "," + str(col) + "\n"

for r in range(row):
    text_str += new_line
    text_str += blank_line
    


with open("./InitMap.txt", "w") as f:
    f.write(text_str)
f.close()
