import sys

with open('astar_core.py', 'r') as f:
    content = f.read()

old_code = """        # Check deadline more frequently (every 500 instead of 2000)
        if expanded % 500 == 0:
            now = time.perf_counter()
            if deadline is not None and now > deadline:
                break
            if deadline is None and (now - t_start) > 20.0:
                break"""

new_code = """        # Check deadline more frequently (every 500 instead of 2000)
        if expanded % 500 == 0:
            now = time.perf_counter()
            if deadline is not None and now > deadline:
                print(f"[A*] Deadline reached! now={now:.2f}, deadline={deadline:.2f}, elapsed={now - t_start:.2f}s")
                break
            if deadline is None and (now - t_start) > 20.0:
                print(f"[A*] 20s timeout reached! elapsed={now - t_start:.2f}s")
                break"""

if old_code in content:
    content = content.replace(old_code, new_code)
    with open('astar_core.py', 'w') as f:
        f.write(content)
    print("Patched astar_core.py successfully!")
else:
    print("Could not find the target code block.")
