# Plotly API Sync

Another way to interact with the plotly data is using their API.
This has various conveniences, such as being able to query all measured switches directly.

**Note**: You'll quickly run out of your daily plotly API usage with a free account using these scripts!


## Requirements

* Python 3
* oauth2client (Python)
* gspread (Python)
* requests (Python)


## Configuration

You'll need to setup a configuration file with the necessary API keys.
By default the configuration file is stored at `~/.config/plotly_gdoc_sync.json`.

For example:

```json
{
        "plotly_query_user" : "haata",
        "plotly_user" : "<plotly username>",
        "plotly_api_key" : "<plotly api key>",
        "gdoc_oauth_json" : "<local google doc oauth json>",
        "gdoc_url" : "https://docs.google.com/spreadsheets/d/<myhash>/edit?usp=sharing",
        "gdoc_scope" : ["https://spreadsheets.google.com/feeds"]
}
```


### Ploty

To generate a Plotly API key, create your own account (Plotly)[https://plot.ly/feed].
Then on the top-right of the Plotly page (your username), click on **Settings**.
On the left click on **API Keys** and click on **Regenerate Keys**.

Use that key as the `plotly_api_key`. The `plotly_user` should be your username.
`plotly_query_user` should be the user you're trying to query.


### Google Doc

See the gspread (documentation)[http://gspread.readthedocs.io/en/latest/oauth2.html] on generating the `gdoc_oauth_json` file.
The `gdoc_url` is just the URL to the google spreadsheet you're trying to synchronize with.


## Usage

```bash
./plotly_gdoc_sync.py --help

```

