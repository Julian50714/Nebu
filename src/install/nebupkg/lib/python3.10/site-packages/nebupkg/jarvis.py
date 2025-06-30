import time
import itertools
from rich.console import Console
from rich.live import Live
from rich.text import Text
from rich.panel import Panel

console = Console()

frames = [
    """
       ⠀⣀⣤⣶⣶⣶⣦⣄⠀⠀
    ⠀⢸⣿⡿⠛⠛⠛⠻⢿⣿⡇
     ⠀⢿⣿⣶⣶⣶⣶⣾⣿⡿
       ⠈⠉⠉⠉⠉⠉⠁⠀
    """,
    """
       ⠀⣴⣶⣾⣿⣷⣶⣦⠀⠀
    ⠀⣿⣿⠟⠋⠉⠉⠙⠻⣿⣿
     ⢿⣿⣷⣶⣶⣶⣾⣿⡿⠁
       ⠈⠁⠈⠉⠁⠈⠁⠀
    """,
    """
       ⠀⣀⣤⣤⣤⣤⣄⠀⠀
    ⠀⣿⣿⠛⠉⠉⠉⠛⣿⣿
     ⠈⢿⣿⣶⣶⣶⣾⡿⠁
       ⠈⠁⠈⠀⠈⠁⠀
    """,
    """
       ⠀⣴⣶⣶⣶⣶⣦⠀⠀
    ⠀⢸⣿⠋⠁⠈⠁⠙⣿⡇
     ⠀⢿⣿⣶⣶⣶⣿⡿⠀
       ⠈⠁⠀⠈⠁⠀⠀
    """
]

def get_frame(frame_str, title="⚡ Energy Portal Active"):
    return Panel(
        Text(frame_str, justify="center"),
        title=f"[bold yellow]{title}",
        border_style="bright_yellow"
    )

def main():
    with Live(console=console, refresh_per_second=5) as live:
        for frame in itertools.cycle(frames):
            panel = get_frame(frame)
            live.update(panel)
            time.sleep(0.2)  # Ajusta para velocidad de animación

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        console.clear()
        console.print("[bold red]✘ Portal cerrado manualmente.")

