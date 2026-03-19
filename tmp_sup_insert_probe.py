import sqlite3

from atlas_adapter.supervisor_daemon import _insert_directive_task

tid = _insert_directive_task(
    title="SUP PROBE",
    detail="probe detail",
    severity="warning",
    signature="probe-signature",
    dedupe_seconds=900,
)
print("inserted_id", tid)

con = sqlite3.connect("C:/ATLAS_PUSH/data/autonomy_tasks.db")
cur = con.cursor()
row = cur.execute(
    "select id,title,action_taken,length(coalesce(action_taken,'')),created_at from autonomy_tasks where id=?",
    (tid,),
).fetchone()
print("row", row)
con.close()
