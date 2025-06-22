import threading

def dummy_function():
    while True:
        pass

threads = []
try:
    while True:
        t = threading.Thread(target=dummy_function)
        threads.append(t)
        t.start()
        print(f"Thread {len(threads)} started")
except Exception as e:
    print(f"Error: {e}")
    print(f"Total threads started: {len(threads)}")
