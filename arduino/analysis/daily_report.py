from datetime import datetime, timedelta
from log_reader import load_log_stats, load_reboot_summary
from email_sender import send_summary_email

def main():
    date = datetime.now() - timedelta(days=1)
    date_str = date.strftime("%Y-%m-%d")

    actual, expected, missing, percentage = load_log_stats(date_str)
    reboot_info = load_reboot_summary(date_str)

    send_summary_email(date_str, actual, expected, missing, percentage, reboot_info)

if __name__ == "__main__":
    main()
