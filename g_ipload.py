from googleapiclient.discovery import build
from httplib2 import Http
from oauth2client import file, client, tools

SCOPES = ['https://www.googleapis.com/auth/drive']

store = file.Storage('token.json')
creds = store.get()
if not creds or creds.invalid:
    flow = client.flow_from_clientsecrets('./client_secret.json', SCOPES)
    creds = tools.run_flow(flow, store)
drive_service = build('drive', 'v3', http=creds.authorize(Http()))
from apiclient.http import MediaFileUpload

#~省略~
drive_service = build('drive', 'v3', http=http_auth)

# '''ここから'''
folder_id = '1uDLwAvSIQYZM9UgjhxXUejx7IO6nkYpY'
file_metadata = {
    'name': 'sample.txt',
    'parents': [folder_id]
}
media = MediaFileUpload(
    'local_path/to/sample.txt', 
    mimetype='text/plain', 
    resumable=True
)
file = drive_service.files().create(
    body=file_metadata, media_body=media, fields='id'
).execute()

#fieldに指定したidをfileから取得できる
print ('File ID: %s' % file.get('id'))
