import functools

# decorator to handle response and print error message
def handle_response(fn):
    @functools.wraps(fn)
    def wrapper(*args, **kwargs):
        res = fn(*args, **kwargs)
        if not res.success:
            print('\033[91mError:\033[0m {}'.format(res.message))
        return res
    return wrapper
