#!python

"""
Demonstration of global and local variables
"""

word = "Global Variable"
# word is defined outside a function, it is a global
# variable that can be accessed in any function

def fun1():
    print(word)
    # word is a global variable, it exists in functions and outside functions
    # word has a scope that is global
    return None

def fun2():
    nonWord = "Local Variable"
    # nonWord was not defined outside the function,
    # it is a local variable and will not exist outside the function
    # it ias a scope that is local
    print(nonWord)
    return None

def fun3():
    word = "Local Variable"
    print(word)
    # word exists globally, and we've defined it locally
    # note that when we use word in the function, we will refer 
    # to the local variable but once we return back to the main 
    # block outside the function, we will revert to using the
    # global version of word
    # this prevents conflicts between local and global variables
    return None

def fun4():
    global word
    word = "changed"
    # Sometimes we want to use the global variable in the function AND make
    # changes to it.  We can declare that the version of word we are going to 
    # use is the global version.  This is probably not good code.
    # it would be better to update the value of the global variable using
    # a return value
    print(word)



if __name__ == "__main__":
    print("===========")
    fun3()
    print(word)

