from fastapi import FastAPI, Request
#from starlette.requests import Request
import json, socket
import uvicorn

robotstate = 'Not Set.'
call = -1

app = FastAPI()

@app.get("/btn_call")
async def get_btn_call():
    global call
    print('call : ' + str(call))
    res = call
    call = -1
    return {"call_id": res}

@app.get("/robotstate")
async def robotstate():
    global robotstate
    #print('root -> robotstate : ' + robotstate)
    return {"message": robotstate}

@app.post('/btn_call')
async def btn_call(info: Request): # var_name: var_type
    global call
    req_info = await info.json()
    print(req_info['call_id'])
    status = ''
    if req_info['call_id'] == 'q':
            status = 'SUCCESS'
    else:
        nCallId = int(req_info['call_id'])
        if nCallId > 0 and nCallId < 4:
            print(nCallId)
            call = nCallId
            status = 'SUCCESS'
        else:
            print('call_id is out of range.')
            status = 'ERROR: OUT OF RANGE'
    return {
        "status" : status,
        "data" : req_info
    }

@app.post('/set_robotstate')
async def set_robotstate(info: Request): # var_name: var_type
    global robotstate
    #print('get_info -> ')
    #print(info)
    req_info = await info.json()
    #print(req_info['state'])
    robotstate = req_info['state']
    #print('robotstate : ' + robotstate)
    status = 'SUCCESS'
    return {
        "status" : status,
        "data" : req_info
    }

