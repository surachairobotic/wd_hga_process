from fastapi import FastAPI, Request
import json, socket
import uvicorn

robotstate = 'Not Set.'

app = FastAPI()

@app.get("/")
async def root():
    global robotstate
    print('root -> robotstate : ' + robotstate)
    return {"message": robotstate}

@app.post('/btn_call')
async def get_info(info: Request): # var_name: var_type
    req_info = await info.json()
    print(req_info['call_id'])
    status = ''
    if req_info['call_id'] == 'q':
            
            status = 'SUCCESS'
    else:
        nCallId = int(req_info['call_id'])
        if nCallId > 0 and nCallId < 4:
            print(nCallId)
            status = 'SUCCESS'
        else:
            print('call_id is out of range.')
            status = 'ERROR: OUT OF RANGE'
    return {
        "status" : status,
        "data" : req_info
    }

@app.post('/set_robotstate')
async def get_info(info: Request): # var_name: var_type
    global robotstate
    req_info = await info.json()
    print(req_info['state'])
    robotstate = req_info['state']
    print('robotstate : ' + robotstate)
    status = 'SUCCESS'
    return {
        "status" : status,
        "data" : req_info
    }

