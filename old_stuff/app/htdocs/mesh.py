import sqlite3


# configuration
DATABASE = '/tmp/flaskr.db'
DEBUG = True
SECRET_KEY = 'development key'
USERNAME = 'admin'
PASSWORD = 'default'


#def connect_db():
#    return sqlite3.connect(app.config['DATABASE'])


#from contextlib import closing
#def init_db():
#    with closing(connect_db()) as db:
#        with app.open_resource('schema.sql', mode='r') as f:
#            db.cursor().executescript(f.read())
#        db.commit()

#@app.before_request
#def before_request():
#    g.db = connect_db()

#@app.teardown_request
#def teardown_request(exception):
#    db = getattr(g, 'db', None)
#    if db is not None:
#        db.close()



db=sqlite3.connect(app.config['DATABASE'])

db.execute('select IMEI_id, latitude, longitude, date, time,' +
                       ' value1, value2, value3 from positions order by id desc')


db.close()

#positions = [dict(IMEI_id=row[0], latitude=row[1], longitude=row[2], 
#                 date=row[3], time=row[4], value1=row[5], 
#                 value2=row[6], value3=row[7]) for row in 

