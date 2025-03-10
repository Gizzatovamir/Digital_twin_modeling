from tenacity import sleep_using_event

from model import Model
import statistics
from pathlib import Path
from model_config import ModelConfig
import yaml
from simple_model import SimpleModel
import datetime
from typing import List, Dict, Callable
import time
import plotly.graph_objs as go

TACT = 0.0001


def down_all_models(model: SimpleModel) -> None:
    if model.layer:
        for child_model in model.layer.models:
            child_model.state = False
            if child_model.layer:
                down_all_models(child_model)


def get_config(path: Path) -> ModelConfig:
    with open(path.as_posix(), "r") as file:
        cfg = yaml.safe_load(file)
        return ModelConfig(**cfg)


def update(func, requset_vector: List[str], mean_TE, t_ME, const_ME, last_ts):
    time.sleep(TACT * mean_TE + const_ME * TACT)
    # time.sleep(TACT * mean_TE)
    func(requset_vector, t_ME * TACT)
    print(f"updated model - {datetime.datetime.now() - last_ts }")


# def model_delayed_update(task: SimpleModel, request_vector: List[str]):
#     time.sleep(TACT * config.mean_TE + config.t_ME * TACT)
#     task.delayed_update(request_vector)
#     print(
#         f"Delayed update for models {' | '.join(request_vector)} is Done, time - {datetime.datetime.now() - last_ts}"
#     )
#
#
# def mode_idle_update(task: SimpleModel):
#     time.sleep(TACT * config.mean_TE + config.t_ME * TACT)
#     task.idle_update()
#     print(
#         f"Idle update for models {' | '.join(request_vector)} is Done, time - {datetime.datetime.now() - last_ts}"
#     )


if __name__ == "__main__":
    config_path: Path = Path("config.yaml")
    config: ModelConfig = get_config(config_path)

    idle_model_list: List[str] = [
        "test_0_1",
        "test_0_0",
        "test_0",
        "test_1_4",
        "test_1_3",
    ]

    request_vector = ["test_0", "test_0_0", "test_0_1", "test_1"]
    test_model = SimpleModel(
        "test", config.NM, config.level_NM, idle_model_list, config.t_MS * TACT
    )
    func_dict: Dict[str, Callable] = {
        "Algorithm 1": test_model.instant_update,
        "Algorithm 2": test_model.delayed_update,
        "Algorithm 3": test_model.idle_update,
    }
    avg_calls = 5
    min_TQ = 1
    max_TQ = 401

    TQ_step = 50

    fig = go.Figure()
    for name, func in func_dict.items():
        x = [el for el in range(1, max_TQ, TQ_step)]
        y = []
        print(f"Algo - {name}")
        for el in x:
            local_y: List[float] = list()
            for i in range(avg_calls):
                last_ts = datetime.datetime.now()
                down_all_models(test_model)
                # test_model.state = test_model.update()
                time.sleep(TACT*config.mean_TE)
                update(
                    func,
                    request_vector,
                    el,
                    config.t_MS,
                    config.mean_TQ,
                    last_ts,
                )
                y_res = (datetime.datetime.now() - last_ts).total_seconds()*1e3
                local_y.append(y_res)
            y.append(statistics.mean(local_y))
            if el % 100 == 1 and el != 1:
                fig.add_annotation(
                    x=el*TACT, y=statistics.mean(local_y),
                    text=f"[{str(el*TACT)[:4]}, {str(statistics.mean(local_y))[:6]}]",
                    showarrow=True,
                    arrowhead=2,
                    font=dict(size=32)
                )
        fig.add_trace(go.Scatter(x=[el*TACT for el in x], y=y, name=name))
    fig.update_layout(
        xaxis_title=dict(text=r"ΔTQ, s", font=dict(size=52)),
        yaxis_title=dict(text="TQ, ms", font=dict(size=52)),
        legend=dict(font=dict(size=52)),
        # xaxis_range=[0, max_TQ],
        xaxis=dict(
            tickfont=dict(size=64)
        ),
        yaxis=dict(
            tickfont=dict(size=64)
        )
    )
    def add_anot(local_x:float, local_y:float, local_string: str):
        fig.add_annotation(text=local_string,
                       align='center',
                       # showarrow=False,
                       xref='paper',
                       yref='paper',
                       x=local_x,
                       y=local_y,
                       bordercolor='black',
                       borderwidth=0.5,
                       font=dict(size=52))

    # add_anot(1.1, 0.65, f"ΔTE - {config.mean_TE*TACT}, s")
    add_anot(1.1, 0.5, f"ΔTQ - {config.mean_TQ*TACT}, s")
    add_anot(1.1, 0.35, f'NM - {config.NM}')
    add_anot(1.1, 0.2, f'NMi - {config.level_NM}')

    fig.show()
    fig.write_html(f"./graphs_x_{len(x)}_mean_TE_{config.mean_TE}.html")
