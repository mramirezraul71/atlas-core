import os, subprocess, sys
os.environ["QUANT_EXIT_GOVERNANCE_ENABLED"] = "false"
os.environ["TRADIER_PAPER_TOKEN"] = "UqYAFhBY0sPWSP4OmuAHChHB0lAN"
venv_py = "C:\\ATLAS_PUSH\\venv\\Scripts\\python.exe"
out_log = "C:\\ATLAS_PUSH\\api_out2.log"
err_log = "C:\\ATLAS_PUSH\\api_err2.log"
p = subprocess.Popen(
    [venv_py, "-m", "uvicorn",
     "atlas_code_quant.api.main:app",
     "--host", "0.0.0.0", "--port", "8795", "--log-level", "info"],
    cwd="C:\\ATLAS_PUSH",
    stdout=open(out_log, "w"),
    stderr=open(err_log, "w"),
    env={**os.environ},
    creationflags=0x00000008
)
print("Launched PID:", p.pid)
