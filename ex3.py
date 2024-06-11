#!python3

import ex31,ex32 


def main():
    ex31.greeting()
    ex32.salutation()
    ex31.salutation()

if __name__ == "__main__":
    main()
    x = 11
    print(f"The value of x is {x}")
    print(f"The value of x in program 1 is {ex31.x}")
    print(f"The value of x in program 2 is {ex32.x}")
    