/**
 * CA 证书 PEM（自签名，用于验证 Mosquitto TLS 服务器证书）
 *
 * 生成方式：
 *   openssl genrsa -out ca.key 2048
 *   openssl req -new -x509 -days 3650 -key ca.key -out ca.crt -subj "/CN=IoT2-CA"
 *
 * 更新方法：替换下方 PEM 字符串内容即可。
 */

const char ca_cert_pem[] =
"-----BEGIN CERTIFICATE-----\n"
"MIIDBTCCAe2gAwIBAgIUGjT8AHgh2JTOIpbfoLHpE/XTR78wDQYJKoZIhvcNAQEL\n"
"BQAwEjEQMA4GA1UEAwwHSW9UMi1DQTAeFw0yNjAyMTkwOTEzMjlaFw0zNjAyMTcw\n"
"OTEzMjlaMBIxEDAOBgNVBAMMB0lvVDItQ0EwggEiMA0GCSqGSIb3DQEBAQUAA4IB\n"
"DwAwggEKAoIBAQCUG9+sqRPhj3MkzP1gFl6yG22YIuiQnsjb2EXnkEtbplOZC3kV\n"
"18+W3p+iOOgvje0lRO/2mG7Kbi2F6V9QOaz0jNCsqV+6SAElzYqwdv5s4qGlbAmh\n"
"XiCn9t/r1V6Ozgs6h/t5ZHh1RUyVD+tyhmmrgK4dRaGbTX0maPYv9pIVkYG+X2Jd\n"
"0h4wAffD0Xm1dyWN+m3NI9XXTQ1bOG6uDmsnRcgMeBA59nHRNmQxr+QYQCCPFpPO\n"
"YElLRbv4m0RHgWjF/6RMrAcPSrCVrNUMYEkogedBNduRS7DuptY2h7TAEjmMf4Gg\n"
"hX3J6rZAQdt/KPKS9L++meTJo2Mfv8jUCKrxAgMBAAGjUzBRMB0GA1UdDgQWBBSi\n"
"3JRLTI1y8ATnJj6GJshGh/3UfDAfBgNVHSMEGDAWgBSi3JRLTI1y8ATnJj6GJshG\n"
"h/3UfDAPBgNVHRMBAf8EBTADAQH/MA0GCSqGSIb3DQEBCwUAA4IBAQAjk4N5TNL5\n"
"wIpyo/LNvPLiQPqfBnIyy1J+mNIDx2xQnSAOiPw1Ym99d36eaNE053b6CaPnp/gX\n"
"ruqVYgYEMHLMJTxOkTD+9qNX4nzA7e49K7uZEJQpGS46UzwiclOLFkKn9NEdCcpc\n"
"bvm0l3UFkfdmfRa2+tbcd+FeesIFX+ljzfC+A5lWFgJ0qrEcAkaTYCCPbuSiC2MM\n"
"vmsC7lYYQfNwSgYDBa+cO3vwA1fwfm3OlyFamRvMTyTVA2fJZHngK3Z8ahWTwNAp\n"
"xVKkqlG1+6WLhY+GroIAq6iJGsO0s2UsB5lsDZsfp2UwOuaqyMMtcLUnqF/maj17\n"
"6UC2jXunQnfS\n"
"-----END CERTIFICATE-----\n";
