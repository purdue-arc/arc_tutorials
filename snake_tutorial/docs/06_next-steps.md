
## Further Subscriber Callback Notes
This is a quick note about dealing with more complex subscriber callbacks. It
isn't required for finishing the tutorial, but you may find this information
useful if you expand upon the basic controller you'll develop. It does present
some somewhat advanced programming concepts, so you may need to do some outside
research if you're not familiar with threading and atomic operations.

In rospy, the callbacks all happen in different threads. Essentially, you're not
guaranteed that one callback will finish before another one get's started. This
can be an issue if you have code like the following:
```python
def callback_one(self, msg):
    self.important_variable = msg.data

def callback_two(self, msg):
    # Check that we can take the log of important_variable
    if self.important_variable > 0:
        some_variable = msg.data * math.log(self.important_variable)
```

What _could_ happen is that `callback_two` gets called and performs the check.
Then `callback_one` gets called and sets `important_variable` to a negative
number. Then, the execution goes back to `callback_two` and we crash the
program since `math.log` throws an error if you try to take the natural
logarithm of a negative number.

You could also have issues with something like iterating through an array if
the array is being changed in a different callback.
