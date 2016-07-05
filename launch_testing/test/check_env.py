if __name__ == '__main__':
    import os
    assert(os.environ.get('testenv2', 'Not Set') == 'Not Set')
    assert(os.environ.get('testenv1', 'Not Set') == 'testval1')
