script_dir="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

mosprotoc -f $script_dir/msg/*/*.msg
mosprotoc -f $script_dir/srv/*/*.srv