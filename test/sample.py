import sys


n = 0


if __name__ == '__main__':
	while True:
		try:
			print(n)
			n+=1
			if(n == 5):
				sys.exit()
		except Exception as e:
			pass
