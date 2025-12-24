import ipaddress
import socket
from datetime import datetime, timedelta, timezone
from pathlib import Path

from cryptography import x509
from cryptography.hazmat.primitives import hashes, serialization
from cryptography.hazmat.primitives.asymmetric import rsa
from cryptography.x509.oid import NameOID


def get_local_ip():
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            s.connect(("8.8.8.8", 80))
            return s.getsockname()[0]
    except Exception:
        return "127.0.0.1"


def generate_self_signed_cert(
    cert_file="dimos/vr/certificates/cert.pem",
    key_file="dimos/vr/certificates/key.pem",
    days_valid=365,
):
    cert_dir = Path(cert_file).parent
    cert_dir.mkdir(exist_ok=True)
    
    local_ip = get_local_ip()
    hostname = socket.gethostname()

    print(f"Generating SSL certificate for {hostname} ({local_ip})")
    
    private_key = rsa.generate_private_key(
        public_exponent=65537,
        key_size=2048,
    )

    subject = issuer = x509.Name([
        x509.NameAttribute(NameOID.COUNTRY_NAME, "US"),
        x509.NameAttribute(NameOID.STATE_OR_PROVINCE_NAME, "CA"),
        x509.NameAttribute(NameOID.LOCALITY_NAME, "Local"),
        x509.NameAttribute(NameOID.ORGANIZATION_NAME, "VR Teleoperation"),
        x509.NameAttribute(NameOID.COMMON_NAME, local_ip),
    ])

    cert = (
        x509.CertificateBuilder()
        .subject_name(subject)
        .issuer_name(issuer)
        .public_key(private_key.public_key())
        .serial_number(x509.random_serial_number())
        .not_valid_before(datetime.now(timezone.utc))
        .not_valid_after(datetime.now(timezone.utc) + timedelta(days=days_valid))
        .add_extension(
            x509.SubjectAlternativeName([
                x509.DNSName("localhost"),
                x509.DNSName(hostname),
                x509.IPAddress(ipaddress.IPv4Address(local_ip)),
                x509.IPAddress(ipaddress.IPv4Address("127.0.0.1")),
            ]),
            critical=False,
        )
        .add_extension(
            x509.BasicConstraints(ca=False, path_length=None),
            critical=True,
        )
        .sign(private_key, hashes.SHA256())
    )

    cert_path = Path(cert_file)
    key_path = Path(key_file)

    with open(key_path, "wb") as f:
        f.write(
            private_key.private_bytes(
                encoding=serialization.Encoding.PEM,
                format=serialization.PrivateFormat.TraditionalOpenSSL,
                encryption_algorithm=serialization.NoEncryption(),
            )
        )

    with open(cert_path, "wb") as f:
        f.write(cert.public_bytes(serialization.Encoding.PEM))

    print(f"SSL certificate generated for {hostname} ({local_ip})")
    print(f"Certificate: {cert_path.absolute()}")
    print(f"Private key: {key_path.absolute()}")


if __name__ == "__main__":
    import sys
    
    cert_path = Path("dimos/vr/certificates/cert.pem")
    key_path = Path("dimos/vr/certificates/key.pem")

    if cert_path.exists() or key_path.exists():
        response = input("Certificate files exist. Overwrite? [y/N]: ")
        if response.lower() != "y":
            sys.exit(0)

    generate_self_signed_cert()
