import random
import string


def get_token():
    # choose from all lowercase letter
    random.seed(None)
    characters = string.ascii_lowercase + string.digits
    token = ''.join(random.choice(characters) for i in range(32))

    return token
