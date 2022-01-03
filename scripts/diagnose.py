import copy
import multiprocessing as mp
from typing import Dict, List, Tuple

import dash
import dash_bootstrap_components as dbc
import numpy as np
import plotly.graph_objects as go
import rospy
from dash import dcc, html
from dash.dependencies import Input, Output
from lio_sam.msg import ObjectState, ObjectStateArray
from std_msgs.msg import Header

# ------------------------------------------------------------------------------------------------ #
#                                      Environmental Variables                                     #
# ------------------------------------------------------------------------------------------------ #

RECENT_HISTORY_LENGTH = 15


# ------------------------------------------------------------------------------------------------ #
#                                               Utils                                              #
# ------------------------------------------------------------------------------------------------ #


def get_color(index):
    colormap = [
        (0.121569, 0.466667, 0.705882),
        (0.682353, 0.780392, 0.909804),
        (1.000000, 0.498039, 0.054902),
        (1.000000, 0.733333, 0.470588),
        (0.172549, 0.627451, 0.172549),
        (0.596078, 0.874510, 0.541176),
        (0.839216, 0.152941, 0.156863),
        (1.000000, 0.596078, 0.588235),
        (0.580392, 0.403922, 0.741176),
        (0.772549, 0.690196, 0.835294),
        (0.549020, 0.337255, 0.294118),
        (0.768627, 0.611765, 0.580392),
        (0.890196, 0.466667, 0.760784),
        (0.968627, 0.713725, 0.823529),
        (0.498039, 0.498039, 0.498039),
        (0.780392, 0.780392, 0.780392),
        (0.737255, 0.741176, 0.133333),
        (0.858824, 0.858824, 0.552941),
        (0.090196, 0.745098, 0.811765),
        (0.619608, 0.854902, 0.898039),
    ]
    colormap = [(int(i * 255), int(j * 255), int(k * 255)) for (i, j, k) in colormap]
    color = colormap[index % 20]
    return "rgba({}, {}, {}, 1)".format(color[0], color[1], color[2])


# ------------------------------------------------------------------------------------------------ #
#                                               Data                                               #
# ------------------------------------------------------------------------------------------------ #


class DynamicObject:
    def __init__(self) -> None:
        self.states: Dict[int, ObjectState] = {}  # {stamp index}: {object state}
        self.index: int = -1
        self.name: str = ""
        self.color: str = "rgba(0, 0, 0, 1)"

    def get_confidence_score(self, index: int) -> float:
        if index not in self.states.keys():
            raise ValueError("Invalid index")
        return self.states[index].detection.value

    def get_initial_tightly_coupled_detection_error(self, index: int) -> float:
        if index not in self.states.keys():
            raise ValueError("Invalid index")
        return self.states[index].initialTightlyCoupledDetectionError

    def get_initial_loosely_coupled_detection_error(self, index: int) -> float:
        if index not in self.states.keys():
            raise ValueError("Invalid index")
        return self.states[index].initialLooselyCoupledDetectionError

    def get_tightly_coupled_detection_error(self, index: int) -> float:
        if index not in self.states.keys():
            raise ValueError("Invalid index")
        return self.states[index].tightlyCoupledDetectionError

    def get_loosely_coupled_detection_error(self, index: int) -> float:
        if index not in self.states.keys():
            raise ValueError("Invalid index")
        return self.states[index].looselyCoupledDetectionError

    def get_initial_motion_error(self, index: int) -> float:
        if index not in self.states.keys():
            raise ValueError("Invalid index")
        return self.states[index].initialMotionError

    def get_motion_error(self, index: int) -> float:
        if index not in self.states.keys():
            raise ValueError("Invalid index")
        return self.states[index].motionError

    def recent_indices(self, start_index: int, value_type: str = "") -> np.ndarray:
        indices = np.array(list(self.states.keys()))
        indices = indices[np.where(indices >= start_index)[0]]
        if value_type in ["", "confidence_score"]:
            return indices
        elif value_type in [
            "initial_tightly_coupled_detection_error",
            "tightly_coupled_detection_error",
        ]:
            return indices[
                np.where(
                    np.array(
                        [
                            1 if self.states[idx].hasTightlyCoupledDetectionError else 0
                            for idx in indices
                        ]
                    )
                )[0]
            ]
        elif value_type in [
            "initial_loosely_coupled_detection_error",
            "loosely_coupled_detection_error",
        ]:
            return indices[
                np.where(
                    np.array(
                        [
                            1 if self.states[idx].hasLooselyCoupledDetectionError else 0
                            for idx in indices
                        ]
                    )
                )[0]
            ]
        elif value_type in ["initial_motion_error", "motion_error"]:
            return indices[
                np.where(
                    np.array(
                        [1 if self.states[idx].hasMotionError else 0 for idx in indices]
                    )
                )[0]
            ]
        else:
            raise ValueError("Invalid value type")

    def has_recent_data(self, start_index: int) -> bool:
        return len(self.recent_indices(start_index)) > 0

    def get_recent_values_plot_data(
        self, value_type: str, start_index: int, stamps: np.ndarray
    ) -> go.Scatter:
        indices = self.recent_indices(start_index, value_type)
        if len(indices) == 0:
            return go.Scatter(x=[], y=[], name=self.name)

        x = stamps[indices]
        y = None
        if value_type == "confidence_score":
            y = [self.states[i].detection.value for i in indices]
        elif value_type == "initial_loosely_coupled_detection_error":
            y = [self.states[i].initialLooselyCoupledDetectionError for i in indices]
        elif value_type == "initial_tightly_coupled_detection_error":
            y = [self.states[i].initialTightlyCoupledDetectionError for i in indices]
        elif value_type == "loosely_coupled_detection_error":
            y = [self.states[i].looselyCoupledDetectionError for i in indices]
        elif value_type == "tightly_coupled_detection_error":
            y = [self.states[i].tightlyCoupledDetectionError for i in indices]
        elif value_type == "initial_motion_error":
            y = [self.states[i].initialMotionError for i in indices]
        elif value_type == "motion_error":
            y = [self.states[i].motionError for i in indices]
        else:
            raise ValueError("Invalid value type")

        return go.Scatter(x=x, y=y, name=self.name, marker=dict(color=self.color))


class Data:
    def __init__(self) -> None:
        self.objects: Dict[int, DynamicObject] = {}
        self.tightly_coupled_objects: Dict[int, DynamicObject] = {}
        self.start_time = -1
        self.stamps: List[float] = []
        self.stamps_hash: List[str] = []

    def add_new_object_state(self, object_state: ObjectState) -> None:
        stamp_index = self.update_time_stamp(object_state.header)

        if object_state.index not in self.objects.keys():
            self.objects[object_state.index] = DynamicObject()
            self.objects[object_state.index].index = object_state.index
            self.objects[object_state.index].name = "Object {}".format(
                object_state.index
            )
            self.objects[object_state.index].color = get_color(object_state.index)

        self.objects[object_state.index].states[stamp_index] = copy.deepcopy(
            object_state
        )

        if object_state.isTightlyCoupled:
            if object_state.index not in self.tightly_coupled_objects.keys():
                self.tightly_coupled_objects[object_state.index] = DynamicObject()
                self.tightly_coupled_objects[
                    object_state.index
                ].index = object_state.index
                self.tightly_coupled_objects[
                    object_state.index
                ].name = "Object {}".format(object_state.index)
                self.tightly_coupled_objects[object_state.index].color = get_color(
                    object_state.index
                )

            self.tightly_coupled_objects[object_state.index].states[
                stamp_index
            ] = copy.deepcopy(object_state)

    def update_time_stamp(self, header: Header) -> int:
        stamp = header.stamp.to_sec()

        if self.start_time < 0:
            self.start_time = stamp

        stamp = stamp - self.start_time
        stamp_hash = "{}.{}".format(header.stamp.secs, header.stamp.nsecs)

        # TODO: Check ordering of time stamps
        if stamp_hash not in self.stamps_hash:
            self.stamps.append(stamp)
            self.stamps_hash.append(stamp_hash)
            return len(self.stamps) - 1
        else:
            length = len(self.stamps)
            return next(
                (
                    length - i - 1
                    for i, e in enumerate(self.stamps_hash[::-1])
                    if e == stamp_hash
                )
            )

    @property
    def number_of_stamps(self):
        return len(self.stamps)

    def get_recent_values_plot_data(
        self, value_type: str, length: int, only_tightly_coupled: bool = False
    ) -> List[go.Scatter]:
        start_index = max(self.number_of_stamps - length, 0)
        np_stamps = np.array(self.stamps)
        available_object_states = (
            self.tightly_coupled_objects.values()
            if only_tightly_coupled
            else self.objects.values()
        )
        return [
            d.get_recent_values_plot_data(value_type, start_index, np_stamps)
            for d in available_object_states
            if d.has_recent_data(start_index)
        ]

    def get_recent_plot_xaxis_range(self, length: int) -> Tuple[float, float]:
        start_index = max(self.number_of_stamps - length, 0)
        end_stamp = max(self.stamps[-1], self.stamps[start_index] + length * 0.1) + 0.2
        return self.stamps[start_index], end_stamp


data = Data()
queue = mp.Queue(maxsize=5)

confidence_score_min_y = 0.0
confidence_score_max_y = 0.55

initial_detection_error_min_y = 0.0
initial_detection_error_max_y = 200
detection_error_min_y = 0.0
detection_error_max_y = 20

initial_motion_error_min_y = 0.0
initial_motion_error_max_y = 5.0
motion_error_min_y = 0.0
motion_error_max_y = 20

# ------------------------------------------------------------------------------------------------ #
#                                                ROS                                               #
# ------------------------------------------------------------------------------------------------ #


class Callback:
    def __init__(self, queue: mp.Queue) -> None:
        self.queue: mp.Queue = queue

    def __call__(self, msg: ObjectStateArray) -> None:
        queue.put(msg)


def ros_main():
    callback = Callback(queue)
    rospy.init_node("lio_sammot_diagnose", anonymous=True)
    rospy.Subscriber("lio_sam/mapping/object_states", ObjectStateArray, callback)
    rospy.spin()


# ------------------------------------------------------------------------------------------------ #
#                                               Dash                                               #
# ------------------------------------------------------------------------------------------------ #

app = dash.Dash(__name__, external_stylesheets=[dbc.themes.FLATLY])
app.layout = html.Div(
    html.Div(
        [
            dbc.NavbarSimple(
                children=[],
                brand="Coupling LIO-SAM and MOT",
                brand_href="#",
                color="primary",
                dark=True,
            ),
            dbc.Toast(
                "Data is reset!",
                id="data-reset-toast",
                header="Notification",
                is_open=False,
                dismissable=True,
                icon="info",
                # top: 66 positions the toast below the navbar
                style={
                    "position": "fixed",
                    "top": 66,
                    "right": 10,
                    "width": 350,
                    "z-index": "100",
                },
            ),
            html.Div(
                [
                    dbc.Row(
                        [
                            dbc.Col(
                                [html.H3("SE-SSD's Confidence Score")],
                                class_name="col-6",
                            ),
                            dbc.Col(
                                [
                                    dbc.InputGroup(
                                        [
                                            dbc.InputGroupText("min(y)"),
                                            dbc.Input(
                                                type="number",
                                                id="confidence-score-min-y",
                                                placeholder="min(y)",
                                                value=confidence_score_min_y,
                                            ),
                                        ]
                                    ),
                                ],
                                class_name="col-3",
                            ),
                            dbc.Col(
                                [
                                    dbc.InputGroup(
                                        [
                                            dbc.InputGroupText("max(y)"),
                                            dbc.Input(
                                                type="number",
                                                id="confidence-score-max-y",
                                                placeholder="max(y)",
                                                value=confidence_score_max_y,
                                            ),
                                        ]
                                    ),
                                ],
                                class_name="col-3",
                            ),
                        ],
                        style={"margin-top": "30px"},
                    ),
                    dbc.Row(
                        [
                            dbc.Col(
                                dcc.Graph(id="detection-score-all"),
                                class_name="col-lg-6 col-12",
                            ),
                            dbc.Col(
                                dcc.Graph(id="detection-score-tightly-coupled"),
                                class_name="col-lg-6 col-12",
                            ),
                        ]
                    ),
                    html.Hr(),
                    dbc.Row(
                        [
                            dbc.Col(
                                [html.H3("Initial Detection Error")], class_name="col-6"
                            ),
                            dbc.Col(
                                [
                                    dbc.InputGroup(
                                        [
                                            dbc.InputGroupText("min(y)"),
                                            dbc.Input(
                                                type="number",
                                                id="initial-detection-error-min-y",
                                                placeholder="min(y)",
                                                value=initial_detection_error_min_y,
                                            ),
                                        ]
                                    ),
                                ],
                                class_name="col-3",
                            ),
                            dbc.Col(
                                [
                                    dbc.InputGroup(
                                        [
                                            dbc.InputGroupText("max(y)"),
                                            dbc.Input(
                                                type="number",
                                                id="initial-detection-error-max-y",
                                                placeholder="max(y)",
                                                value=initial_detection_error_max_y,
                                            ),
                                        ]
                                    ),
                                ],
                                class_name="col-3",
                            ),
                        ],
                        style={"margin-top": "30px"},
                    ),
                    dbc.Row(
                        [
                            dbc.Col(
                                dcc.Graph(id="initial-loosely-coupled-detection-error"),
                                class_name="col-lg-6 col-12",
                            ),
                            dbc.Col(
                                dcc.Graph(id="initial-tightly-coupled-detection-error"),
                                class_name="col-lg-6 col-12",
                            ),
                        ]
                    ),
                    html.Hr(),
                    dbc.Row(
                        [
                            dbc.Col([html.H3("Detection Error")], class_name="col-6"),
                            dbc.Col(
                                [
                                    dbc.InputGroup(
                                        [
                                            dbc.InputGroupText("min(y)"),
                                            dbc.Input(
                                                type="number",
                                                id="detection-error-min-y",
                                                placeholder="min(y)",
                                                value=detection_error_min_y,
                                            ),
                                        ]
                                    ),
                                ],
                                class_name="col-3",
                            ),
                            dbc.Col(
                                [
                                    dbc.InputGroup(
                                        [
                                            dbc.InputGroupText("max(y)"),
                                            dbc.Input(
                                                type="number",
                                                id="detection-error-max-y",
                                                placeholder="max(y)",
                                                value=detection_error_max_y,
                                            ),
                                        ]
                                    ),
                                ],
                                class_name="col-3",
                            ),
                        ],
                        style={"margin-top": "30px"},
                    ),
                    dbc.Row(
                        [
                            dbc.Col(
                                dcc.Graph(id="loosely-coupled-detection-error"),
                                class_name="col-lg-6 col-12",
                            ),
                            dbc.Col(
                                dcc.Graph(id="tightly-coupled-detection-error"),
                                class_name="col-lg-6 col-12",
                            ),
                        ]
                    ),
                    # html.Hr(),
                    # dbc.Row(
                    #     [
                    #         dbc.Col(
                    #             [html.H3("Initial Motion Error")], class_name="col-6"
                    #         ),
                    #         dbc.Col(
                    #             [
                    #                 dbc.InputGroup(
                    #                     [
                    #                         dbc.InputGroupText("min(y)"),
                    #                         dbc.Input(
                    #                             type="number",
                    #                             id="initial-motion-error-min-y",
                    #                             placeholder="min(y)",
                    #                             value=initial_motion_error_min_y,
                    #                         ),
                    #                     ]
                    #                 ),
                    #             ],
                    #             class_name="col-3",
                    #         ),
                    #         dbc.Col(
                    #             [
                    #                 dbc.InputGroup(
                    #                     [
                    #                         dbc.InputGroupText("max(y)"),
                    #                         dbc.Input(
                    #                             type="number",
                    #                             id="initial-motion-error-max-y",
                    #                             placeholder="max(y)",
                    #                             value=initial_motion_error_max_y,
                    #                         ),
                    #                     ]
                    #                 ),
                    #             ],
                    #             class_name="col-3",
                    #         ),
                    #     ],
                    #     style={"margin-top": "30px"},
                    # ),
                    # dbc.Row(
                    #     [
                    #         dbc.Col(
                    #             dcc.Graph(id="initial-motion-error-all"),
                    #             class_name="col-lg-6 col-12",
                    #         ),
                    #         dbc.Col(
                    #             dcc.Graph(id="initial-motion-error-tightly-coupled"),
                    #             class_name="col-lg-6 col-12",
                    #         ),
                    #     ]
                    # ),
                    html.Hr(),
                    dbc.Row(
                        [
                            dbc.Col([html.H3("Motion Error")], class_name="col-6"),
                            dbc.Col(
                                [
                                    dbc.InputGroup(
                                        [
                                            dbc.InputGroupText("min(y)"),
                                            dbc.Input(
                                                type="number",
                                                id="motion-error-min-y",
                                                placeholder="min(y)",
                                                value=motion_error_min_y,
                                            ),
                                        ]
                                    ),
                                ],
                                class_name="col-3",
                            ),
                            dbc.Col(
                                [
                                    dbc.InputGroup(
                                        [
                                            dbc.InputGroupText("max(y)"),
                                            dbc.Input(
                                                type="number",
                                                id="motion-error-max-y",
                                                placeholder="max(y)",
                                                value=motion_error_max_y,
                                            ),
                                        ]
                                    ),
                                ],
                                class_name="col-3",
                            ),
                        ],
                        style={"margin-top": "30px"},
                    ),
                    dbc.Row(
                        [
                            dbc.Col(
                                dcc.Graph(id="motion-error-all"),
                                class_name="col-lg-6 col-12",
                            ),
                            dbc.Col(
                                dcc.Graph(id="motion-error-tightly-coupled"),
                                class_name="col-lg-6 col-12",
                            ),
                        ]
                    ),
                    html.Hr(),
                    html.Div(
                        [
                            dbc.Button(
                                "Reset",
                                id="reset-button",
                                color="danger",
                                n_clicks=0,
                                className="me-1",
                            ),
                        ]
                    ),
                ],
                className="container",
            ),
            dcc.Interval(
                id="interval-component",
                interval=1 * 200,  # in milliseconds
                n_intervals=0,
            ),
        ]
    )
)


@app.callback(
    Output("data-reset-toast", "is_open"),
    Input("reset-button", "n_clicks"),
)
def reset(n):
    global data
    data = Data()
    return True


@app.callback(
    Output("detection-score-all", "figure"),
    Output("detection-score-tightly-coupled", "figure"),
    Output("initial-loosely-coupled-detection-error", "figure"),
    Output("initial-tightly-coupled-detection-error", "figure"),
    Output("loosely-coupled-detection-error", "figure"),
    Output("tightly-coupled-detection-error", "figure"),
    # Output("initial-motion-error-all", "figure"),
    # Output("initial-motion-error-tightly-coupled", "figure"),
    Output("motion-error-all", "figure"),
    Output("motion-error-tightly-coupled", "figure"),
    Input("interval-component", "n_intervals"),
    Input("confidence-score-min-y", "value"),
    Input("confidence-score-max-y", "value"),
    Input("initial-detection-error-min-y", "value"),
    Input("initial-detection-error-max-y", "value"),
    Input("detection-error-min-y", "value"),
    Input("detection-error-max-y", "value"),
    # Input("initial-motion-error-min-y", "value"),
    # Input("initial-motion-error-max-y", "value"),
    Input("motion-error-min-y", "value"),
    Input("motion-error-max-y", "value"),
)
def update(
    n,
    c_min_y,
    c_max_y,
    i_d_min_y,
    i_d_max_y,
    d_min_y,
    d_max_y,
    # i_m_min_y,
    # i_m_max_y,
    m_min_y,
    m_max_y,
):
    global confidence_score_min_y
    global confidence_score_max_y
    global initial_detection_error_min_y
    global initial_detection_error_max_y
    global detection_error_min_y
    global detection_error_max_y
    global initial_motion_error_min_y
    global initial_motion_error_max_y
    global motion_error_min_y
    global motion_error_max_y

    changed = False

    if confidence_score_min_y != c_min_y:
        changed = True
        confidence_score_min_y = c_min_y
    if confidence_score_max_y != c_max_y:
        changed = True
        confidence_score_max_y = c_max_y
    if initial_detection_error_min_y != i_d_min_y:
        changed = True
        initial_detection_error_min_y = i_d_min_y
    if initial_detection_error_max_y != i_d_max_y:
        changed = True
        initial_detection_error_max_y = i_d_max_y
    if detection_error_min_y != d_min_y:
        changed = True
        detection_error_min_y = d_min_y
    if detection_error_max_y != d_max_y:
        changed = True
        detection_error_max_y = d_max_y
    # if initial_motion_error_min_y != i_m_min_y:
    #     changed = True
    #     initial_motion_error_min_y = i_m_min_y
    # if initial_motion_error_max_y != i_m_max_y:
    #     changed = True
    #     initial_motion_error_max_y = i_m_max_y
    if motion_error_min_y != m_min_y:
        changed = True
        motion_error_min_y = m_min_y
    if motion_error_max_y != m_max_y:
        changed = True
        motion_error_max_y = m_max_y

    if queue.empty() and not changed:
        return [dash.no_update] * 8
    elif changed and len(data.objects) == 0:
        return [dash.no_update] * 8

    while not queue.empty():
        msg: ObjectStateArray = queue.get()
        for object_state in msg.objects:
            data.add_new_object_state(object_state)

    confidence_score_all = go.Figure(
        data=[
            go.Scatter(
                x=[data.stamps[-1]] * 2,
                y=[confidence_score_min_y, confidence_score_max_y],
                marker={"color": "red"},
            )
        ]
        + data.get_recent_values_plot_data(
            "confidence_score", RECENT_HISTORY_LENGTH, False
        ),
        layout_xaxis_range=data.get_recent_plot_xaxis_range(RECENT_HISTORY_LENGTH),
        layout_yaxis_range=[0, 0.55],
    )
    confidence_score_all.update_layout(
        title="All Objects",
        xaxis_title="Time Stamp",
        yaxis_title="Confidence Score",
        showlegend=True,
    )
    confidence_score_tightly_coupled = go.Figure(
        data=[
            go.Scatter(
                x=[data.stamps[-1]] * 2,
                y=[confidence_score_min_y - 1, confidence_score_max_y + 1],
                marker={"color": "red"},
            )
        ]
        + data.get_recent_values_plot_data(
            "confidence_score", RECENT_HISTORY_LENGTH, True
        ),
        layout_xaxis_range=data.get_recent_plot_xaxis_range(RECENT_HISTORY_LENGTH),
        layout_yaxis_range=[confidence_score_min_y, confidence_score_max_y],
    )
    confidence_score_tightly_coupled.update_layout(
        title="Tightly-coupled Objects",
        xaxis_title="Time Stamp",
        yaxis_title="Confidence Score",
        showlegend=True,
    )

    initial_loosely_coupled_detection_error = go.Figure(
        data=[
            go.Scatter(
                x=[data.stamps[-1]] * 2,
                y=[
                    initial_detection_error_min_y - 1,
                    initial_detection_error_max_y + 1,
                ],
                marker={"color": "red"},
            )
        ]
        + data.get_recent_values_plot_data(
            "initial_loosely_coupled_detection_error", RECENT_HISTORY_LENGTH, False
        ),
        layout_xaxis_range=data.get_recent_plot_xaxis_range(RECENT_HISTORY_LENGTH),
        layout_yaxis_range=[
            initial_detection_error_min_y,
            initial_detection_error_max_y,
        ],
    )
    initial_loosely_coupled_detection_error.update_layout(
        title="Loosely-coupled Objects",
        xaxis_title="Time Stamp",
        yaxis_title="Loosely-coupled Detection Error",
        showlegend=True,
    )
    initial_tightly_coupled_detection_error = go.Figure(
        data=[
            go.Scatter(
                x=[data.stamps[-1]] * 2,
                y=[
                    initial_detection_error_min_y - 1,
                    initial_detection_error_max_y + 1,
                ],
                marker={"color": "red"},
            )
        ]
        + data.get_recent_values_plot_data(
            "initial_tightly_coupled_detection_error", RECENT_HISTORY_LENGTH, False
        ),
        layout_xaxis_range=data.get_recent_plot_xaxis_range(RECENT_HISTORY_LENGTH),
        layout_yaxis_range=[
            initial_detection_error_min_y,
            initial_detection_error_max_y,
        ],
    )
    initial_tightly_coupled_detection_error.update_layout(
        title="Tightly-coupled Objects",
        xaxis_title="Time Stamp",
        yaxis_title="Tightly-coupled Detection Error",
        showlegend=True,
    )

    loosely_coupled_detection_error = go.Figure(
        data=[
            go.Scatter(
                x=[data.stamps[-1]] * 2,
                y=[detection_error_min_y - 1, detection_error_max_y + 1],
                marker={"color": "red"},
            )
        ]
        + data.get_recent_values_plot_data(
            "loosely_coupled_detection_error", RECENT_HISTORY_LENGTH, False
        ),
        layout_xaxis_range=data.get_recent_plot_xaxis_range(RECENT_HISTORY_LENGTH),
        layout_yaxis_range=[detection_error_min_y, detection_error_max_y],
    )
    loosely_coupled_detection_error.update_layout(
        title="Loosely-coupled Objects",
        xaxis_title="Time Stamp",
        yaxis_title="Loosely-coupled Detection Error",
        showlegend=True,
    )
    tightly_coupled_detection_error = go.Figure(
        data=[
            go.Scatter(
                x=[data.stamps[-1]] * 2,
                y=[detection_error_min_y - 1, detection_error_max_y + 1],
                marker={"color": "red"},
            )
        ]
        + data.get_recent_values_plot_data(
            "tightly_coupled_detection_error", RECENT_HISTORY_LENGTH, False
        ),
        layout_xaxis_range=data.get_recent_plot_xaxis_range(RECENT_HISTORY_LENGTH),
        layout_yaxis_range=[detection_error_min_y, detection_error_max_y],
    )
    tightly_coupled_detection_error.update_layout(
        title="Tightly-coupled Objects",
        xaxis_title="Time Stamp",
        yaxis_title="Tightly-coupled Detection Error",
        showlegend=True,
    )

    # initial_motion_error_all = go.Figure(
    #     data=[
    #         go.Scatter(
    #             x=[data.stamps[-1]] * 2,
    #             y=[initial_motion_error_min_y - 1, initial_motion_error_max_y + 1],
    #             marker={"color": "red"},
    #         )
    #     ]
    #     + data.get_recent_values_plot_data(
    #         "initial_motion_error", RECENT_HISTORY_LENGTH, False
    #     ),
    #     layout_xaxis_range=data.get_recent_plot_xaxis_range(RECENT_HISTORY_LENGTH),
    #     layout_yaxis_range=[initial_motion_error_min_y, initial_motion_error_max_y],
    # )
    # initial_motion_error_all.update_layout(
    #     title="All Objects",
    #     xaxis_title="Time Stamp",
    #     yaxis_title="Motion Error",
    #     showlegend=True,
    # )
    # initial_motion_error_tightly_coupled = go.Figure(
    #     data=[
    #         go.Scatter(
    #             x=[data.stamps[-1]] * 2,
    #             y=[initial_motion_error_min_y - 1, initial_motion_error_max_y + 1],
    #             marker={"color": "red"},
    #         )
    #     ]
    #     + data.get_recent_values_plot_data(
    #         "initial_motion_error", RECENT_HISTORY_LENGTH, True
    #     ),
    #     layout_xaxis_range=data.get_recent_plot_xaxis_range(RECENT_HISTORY_LENGTH),
    #     layout_yaxis_range=[initial_motion_error_min_y, initial_motion_error_max_y],
    # )
    # initial_motion_error_tightly_coupled.update_layout(
    #     title="Tightly-coupled Objects",
    #     xaxis_title="Time Stamp",
    #     yaxis_title="Motion Error",
    #     showlegend=True,
    # )

    motion_error_all = go.Figure(
        data=[
            go.Scatter(
                x=[data.stamps[-1]] * 2,
                y=[motion_error_min_y - 1, motion_error_max_y + 1],
                marker={"color": "red"},
            )
        ]
        + data.get_recent_values_plot_data(
            "motion_error", RECENT_HISTORY_LENGTH, False
        ),
        layout_xaxis_range=data.get_recent_plot_xaxis_range(RECENT_HISTORY_LENGTH),
        layout_yaxis_range=[motion_error_min_y, motion_error_max_y],
    )
    motion_error_all.update_layout(
        title="All Objects",
        xaxis_title="Time Stamp",
        yaxis_title="Motion Error",
        showlegend=True,
    )
    motion_error_tightly_coupled = go.Figure(
        data=[
            go.Scatter(
                x=[data.stamps[-1]] * 2,
                y=[motion_error_min_y - 1, motion_error_max_y + 1],
                marker={"color": "red"},
            )
        ]
        + data.get_recent_values_plot_data("motion_error", RECENT_HISTORY_LENGTH, True),
        layout_xaxis_range=data.get_recent_plot_xaxis_range(RECENT_HISTORY_LENGTH),
        layout_yaxis_range=[motion_error_min_y, motion_error_max_y],
    )
    motion_error_tightly_coupled.update_layout(
        title="Tightly-coupled Objects",
        xaxis_title="Time Stamp",
        yaxis_title="Motion Error",
        showlegend=True,
    )

    return (
        confidence_score_all,
        confidence_score_tightly_coupled,
        initial_loosely_coupled_detection_error,
        initial_tightly_coupled_detection_error,
        loosely_coupled_detection_error,
        tightly_coupled_detection_error,
        # initial_motion_error_all,
        # initial_motion_error_tightly_coupled,
        motion_error_all,
        motion_error_tightly_coupled,
    )


if __name__ == "__main__":
    ros_main_process = mp.Process(target=ros_main)
    ros_main_process.start()
    app.run_server(debug=True)
    ros_main_process.join()
