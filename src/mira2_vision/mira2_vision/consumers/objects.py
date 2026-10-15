from vision_msgs.msg import Detection2DArray
from mira2_vision.plugin import Plugin


class Objects(Plugin):
    requires = ('detections_2d',)
    after = ('depth_association',)
    provides = ()

    def __init__(self, node):
        super().__init__(node)
        self.publisher = node.create_publisher(Detection2DArray, 'objects', 10)

    def process(self, context):
        self.publisher.publish(context.products['detections_2d'])
