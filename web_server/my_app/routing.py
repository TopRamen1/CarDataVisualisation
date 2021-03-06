# chat/routing.py
from django.urls import re_path

from my_app.Robot.RobotConsumer import RobotConsumer

websocket_urlpatterns = [
    re_path(r'ws/robot/(?P<room_name>\w+)/$', RobotConsumer),
]
