from hm_2.math_tools import get_flat_circle_params, get_circle_points_arr, Point


def test_get_flat_circle_params():

    print('test_get_flat_circle_params')
    a = Point(0, 5, 0)
    b = Point(-5, 0, 0)
    c = Point(5, 0, 0)
    print(get_flat_circle_params(a, b, c))

    a = Point(5, 5, 0)
    b = Point(5, -5, 0)
    c = Point(10, 0, 0)
    print(get_flat_circle_params(a, b, c))

    a = Point(0, 1, 0)
    b = Point(1, 0, 0)
    c = Point(2, 1, 0)
    print(get_flat_circle_params(a, b, c))


def test_get_circle_points_arr():

    print('test_get_circle_points_arr')
    a = Point(0, 1, 0)
    b = Point(1, 0, 0)
    c = Point(2, 1, 0)

    circle_points = get_circle_points_arr(
        circle_params=get_flat_circle_params(a, b, c),
        num=4,
    )

    print(circle_points)

if __name__ == '__main__':

    test_get_flat_circle_params()
    test_get_circle_points_arr()
