import threading
import time

# Define a function that runs in the thread and takes an argument
def thread_function(data):
    while not stop_thread:
        print(f"Thread is running with data: {data}")
        time.sleep(1)

# Create a global variable to control the thread
stop_thread = False

# Define the specific data you want to pass to the thread
thread_data = "Hello, Thread!"

# Create a thread and pass the data as an argument
thread = threading.Thread(target=thread_function, args=(thread_data,))

# Start the thread
thread.start()

# Wait for a few seconds
time.sleep(5)

# Set the flag to stop the thread
stop_thread = True

# Wait for the thread to finish
thread.join()

print("Thread has been stopped.")
