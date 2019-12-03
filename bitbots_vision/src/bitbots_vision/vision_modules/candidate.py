import abc
import rospy


class Candidate:
    def __init__(self, x1=0, y1=0, width=0, height=0, rating=None):
        self._x1 = x1
        self._y1 = y1
        self._width = width
        self._height = height
        self._rating = rating

    def get_width(self):
        # type: () -> int
        """
        :return int: Width of the candidate bounding box.
        """
        return self._width

    def get_height(self):
        # type: () -> int
        """
        :return int: Height of the candidate bounding box.
        """
        return self._height

    def get_center_x(self):
        # type: () -> int
        """
        :return int: Center x coordinate of the candidate bounding box.
        """
        return self._x1 + int(self._width // 2)

    def get_center_y(self):
        # type: () -> int
        """
        :return int: Center y coordinate of the candidate bounding box.
        """
        return self._y1 + int(self._height // 2)

    def get_center_point(self):
        # type: () -> tuple[int, int]
        """
        :return tuple[int,int]: Center point of the bounding box.
        """
        return self.get_center_x(), self.get_center_y()

    def get_diameter(self):
        # type: () -> int
        """
        :return int: Mean diameter of the candidate.
        """
        return int((self._height + self._width) // 2)

    def get_radius(self):
        # type: () -> int
        """
        :return int: Mean radius of the candidate.
        """
        return int(self.get_diameter() // 2)

    def get_upper_left_point(self):
        # type: () -> tuple[int, int]
        """
        :return tuple[int,int]: Upper left point of the candidate.
        """
        return self._x1, self._y1

    def get_upper_left_x(self):
        # type: () -> int
        """
        :return int: Upper left x coordinate of the candidate.
        """
        return self._x1

    def get_upper_left_y(self):
        # type: () -> int
        """
        :return int: Upper left y coordinate of the candidate.
        """
        return self._y1

    def get_lower_right_point(self):
        # type: () -> tuple[int, int]
        """
        :return tuple[int,int]: Lower right point of the candidate.
        """
        return self._x1 + self._width, self._y1 + self._height

    def get_lower_right_x(self):
        # type: () -> int
        """
        :return int: Lower right x coordinate of the candidate.
        """
        return self._x1 + self._width

    def get_lower_right_y(self):
        # type: () -> int
        """
        :return int: Lower right y coordinate of the candidate.
        """
        return self._y1 + self._height

    def get_lower_center_point(self):
        # type: () -> (int, int)
        """
        :return tuple: Returns the lowest point of the candidate. The point is horizontally centered inside the candidate.
        """
        return (self.get_center_x(), self.get_lower_right_y())

    def set_rating(self, rating):
        # type: (float) -> None
        """
        :param float rating: Rating to set.
        """
        if self._rating is not None:
            rospy.logwarn('Candidate rating has already been set.', logger_name='Candidate')
            return
        self._rating = rating

    def get_rating(self):
        # type: () -> float
        """

        :return float: Rating of the candidate
        """
        return self._rating

    def point_in_candidate(self, point):
        # type: (tuple) -> bool
        """
        Returns whether the point is in the candidate or not.
        In the process, the candidate gets treated as a rectangle.

        :param point: An x-y-int-tuple defining the point to inspect.
        :return bool: Whether the point is in the candidate or not.
        """
        return (
                self.get_upper_left_x()
                <= point[0]
                <= self.get_upper_left_x() + self.get_width()) \
            and (
                self.get_upper_left_y()
                <= point[1]
                <= self.get_upper_left_y() + self.get_height())

    @staticmethod
    def sort_candidates(candidatelist):
        """
        Returns a sorted list of the candidates.
        The first list element is the highest rated candidate.

        :param candidatelist: List of candidate objects.
        :return: Sorted list of candidate objects.
        """
        return sorted(candidatelist, key = lambda candidate: candidate.get_rating(), reverse=True)

    @staticmethod
    def select_top_candidate(candidatelist):
        if candidatelist:
            return Candidate.sort_candidates(candidatelist)[0]
        else:
            return None

    @staticmethod
    def rating_threshold(candidatelist, threshold):
        return [candidate for candidate in candidatelist if candidate.get_rating() > threshold]

    def __str__(self):
        return 'x1,y1: {0},{1} | width,height: {2},{3} | rating: {4}'.format(
            self.get_upper_left_x(),
            self.get_upper_left_y(),
            self.get_width(),
            self.get_height(),
            self._rating)


class CandidateFinder(object):
    """
    Abstract definition of a CandidateFinder.
    """
    def get_top_candidates(self, count=1):
        """
        Returns the count best candidates.

        :param count: Number of top-candidates to return
        :return: the count top candidates
        """
        ball_candidates = self.get_candidates()
        ball_candidates = Candidate.sort_candidates(ball_candidates)
        return ball_candidates[:count]

    @abc.abstractmethod
    def get_candidates(self):
        """
        Returns a list of all candidates. Their type is Candidate.

        :return: the count top candidates
        """
        raise NotImplementedError

    @abc.abstractmethod
    def compute(self):
        """
        Runs the most intense calculation without returning any output and caches the result.
        """
        raise NotImplementedError

    def get_top_candidate(self):
        """
        Returns the best candidate.

        :return: the best candidate or nothing
        """
        return Candidate.select_top_candidate(self.get_candidates())

class BallDetector(CandidateFinder):
    def __init__(self):
        pass

