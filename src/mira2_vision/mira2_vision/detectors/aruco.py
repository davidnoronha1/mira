from utils.parameters import Setting, declare_config, positive
import time
import cv2
from mira2_vision.plugin import Plugin


class Aruco(Plugin):
    requires = ()
    provides = ('aruco',)

    def __init__(self, node):
        super().__init__(node)
        self.config = declare_config(node, {
            'dictionary': "DICT_ARUCO_ORIGINAL",
            'clahe_clip_limit': Setting(2.5, positive),
            'clahe_grid_size': Setting(8, positive),
            'rate': Setting(30.0, positive),
        }, namespace='aruco')
        self.aruco = None
        self.clahe = None
        dictionary_name = self.config["dictionary"]
        if not hasattr(cv2.aruco, dictionary_name):
            raise ValueError(f"Unknown ArUco dictionary: {dictionary_name}")
        dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dictionary_name))
        parameters = (cv2.aruco.DetectorParameters() if hasattr(cv2.aruco, "ArucoDetector")
                      else cv2.aruco.DetectorParameters_create())
        parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        if hasattr(cv2.aruco, "ArucoDetector"):
            detector = cv2.aruco.ArucoDetector(dictionary, parameters)
            self.aruco = detector.detectMarkers
        else:
            self.aruco = lambda image: cv2.aruco.detectMarkers(image, dictionary, parameters=parameters)
        self.clahe = cv2.createCLAHE(self.config["clahe_clip_limit"],
                                   (self.config["clahe_grid_size"],) * 2)

        self.last_run = 0.0

    def process(self, context):
        now = time.monotonic()
        if now - self.last_run < 1 / self.config['rate']:
            return
        self.last_run = now
        gray = self.clahe.apply(cv2.cvtColor(context.image, cv2.COLOR_BGR2GRAY))
        corners, ids, _ = self.aruco(gray)
        visible = [] if ids is None else [int(mid) for mid in ids.flatten()]
        context.products['aruco'] = (corners, visible)
        if context.preview is not None and ids is not None:
            cv2.aruco.drawDetectedMarkers(context.preview, corners, ids)
