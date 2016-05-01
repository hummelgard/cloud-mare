chdir = "/srv/http/mundilfare/"
#worker_class = "sync"
worker_class = "gevent"
#worker_connections ="2"
errorlog = "log/mundilfare.gunicorn.log"
pid = "run/pid"

# LOG LEVELS
loglevel = "info"
#debug
#info
#warning
#error
#critical

