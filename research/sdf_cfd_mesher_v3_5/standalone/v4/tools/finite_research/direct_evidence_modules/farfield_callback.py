import math

class CheckedCallback:
    def __init__(self,measured):
        self.measured=measured
        self.failure=None
        self.returned=0

    def __call__(self,*arguments):
        try:
            value=self.measured(*arguments)
            if not math.isfinite(value) or value<=0:
                raise ValueError("InvalidFarfieldSize")
            self.returned+=1
            return value
        except BaseException as error:
            if self.failure is None:
                self.failure=repr(error)
            return float("nan")

    def require(self):
        if self.failure is not None or self.returned<=0:
            raise ValueError("FarfieldCallbackFailedOrInactive",self.failure)
        return dict(self.measured.report(),positive_finite_returns=self.returned,failure=None)
