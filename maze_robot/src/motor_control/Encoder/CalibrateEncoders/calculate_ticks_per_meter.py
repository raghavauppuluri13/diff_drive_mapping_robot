file_n = 'encoder_config.txt'
f = open(file_n, "r")
tot = 0
num_lines = sum(1 for line in open(file_n))
for x in f:
	l,r = x.split(',')
	av = (int(l)+int(r))/2.0
	tot = tot + av
print(tot/num_lines)
